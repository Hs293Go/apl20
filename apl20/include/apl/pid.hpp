#ifndef APL_PID_HPP_
#define APL_PID_HPP_

#include <Eigen/Core>
#include <concepts>
#include <limits>

#include "apl/filters.hpp"

namespace apl {

// Tuning for an N-axis bank of independent PID loops. Pure "bag of data":
// default-constructs to a safe no-op (all gains zero, no limits, no filtering)
// and is populated with designated initializers. Every member is a per-axis
// Eigen array so the kernel stays branch-free and vectorized.
template <std::floating_point Scalar, int N = 1>
struct PidCfg {
  using Array = Eigen::Array<Scalar, N, 1>;

  Array kp = Array::Zero();   // proportional gain (acts on error)
  Array ki = Array::Zero();   // integral gain
  Array kd = Array::Zero();   // derivative gain (acts on measurement)
  Array kff = Array::Zero();  // setpoint feed-forward gain

  // Absolute clamp on the integrator state (hard anti-windup). Default
  // unbounded; a real controller sets a finite value.
  Array i_max = Array::Constant(std::numeric_limits<Scalar>::infinity());

  // Cutoff [Hz] of the first-order low-pass on the derivative. The raw
  // measurement derivative is noisy, so flight stacks always filter it. Default
  // is unbounded (no filtering); the driver sets a real cutoff.
  Array d_lpf_hz = Array::Constant(std::numeric_limits<Scalar>::infinity());

  // Error magnitude at which integral authority tapers to zero, following PX4's
  // `i_factor = max(0, 1 - (error / i_decay_error)^2)`. This bleeds off
  // integral action during large transients (e.g. flips) to avoid bounce-back.
  // Default unbounded => taper disabled.
  Array i_decay_error =
      Array::Constant(std::numeric_limits<Scalar>::infinity());
};

// Evolving state of the loop bank: the only mutable data in the whole design.
// The driver owns one of these and threads it through `Pid::update`, so the Pid
// object itself stays stateless and const — the explicit-state inversion of the
// Arduino library's pointer-and-millis() approach.
template <std::floating_point Scalar, int N = 1>
struct PidState {
  using Array = Eigen::Array<Scalar, N, 1>;

  Array integrator = Array::Zero();  // accumulated integral term
  Array prev_meas = Array::Zero();   // measurement at the previous step
  Array d_term = Array::Zero();      // state of the derivative low-pass filter
  bool seeded = false;  // false until the first sample primes prev_meas
};

// Decomposed controller output, one column per term — the equivalent of
// ArduPilot's AP_PIDInfo. Returned by value from every update so the term split
// is available for logging and tuning without reaching into the controller.
template <std::floating_point Scalar, int N = 1>
struct PidTerms {
  using Array = Eigen::Array<Scalar, N, 1>;

  Array p = Array::Zero();
  Array i = Array::Zero();
  Array d = Array::Zero();
  Array ff = Array::Zero();

  Array output() const { return p + i + d + ff; }
};

// Per-update modifiers that the driver supplies but that are conceptually pure
// math: gain scheduling and saturation-aware anti-windup. Defaults are neutral
// (unity scaling, no saturation) so the simple call `update(state, sp, m, dt)`
// is a plain PID step.
template <std::floating_point Scalar, int N = 1>
struct PidModifiers {
  using Array = Eigen::Array<Scalar, N, 1>;
  using Mask = Eigen::Array<bool, N, 1>;

  // Per-axis gain scheduling (ArduPilot's pd_scale / i_scale). The driver maps
  // throttle PID attenuation, gain scheduling, etc. onto these.
  Array pd_scale = Array::Ones();
  Array i_scale = Array::Ones();

  // Output-saturation flags from the actuator stage. `sat_hi` blocks the
  // integrator from growing further positive, `sat_lo` from growing further
  // negative — conditional integration, the anti-windup scheme shared by PX4
  // (control-allocator feedback) and ArduPilot (the `limit` flag).
  Mask sat_hi = Mask::Zero();
  Mask sat_lo = Mask::Zero();
};

// Stateless N-axis PID math kernel. Holds only its `PidCfg` and every method is
// const; all evolving data lives in the caller-owned `PidState`. Knows nothing
// about drones, time sources, actuators, or ROS — that all lives in the driver.
template <std::floating_point Scalar, int N = 1>
class Pid {
  static_assert(N > 0, "PID needs at least one axis");

 public:
  using Array = Eigen::Array<Scalar, N, 1>;

  explicit Pid(const PidCfg<Scalar, N>& cfg)
      : cfg_(cfg), d_lpf_(cfg.d_lpf_hz) {}

  const PidCfg<Scalar, N>& cfg() const { return cfg_; }

  // Prime the loop from a fresh measurement: zero the integrator and derivative
  // history so the next update emits no derivative spike. Call on (re)start.
  void reset(PidState<Scalar, N>& state,
             const Eigen::Ref<const Array>& meas) const {
    state.integrator.setZero();
    state.prev_meas = meas;
    state.d_term.setZero();
    state.seeded = true;
  }

  // One discrete PID step. Precondition: dt > 0 (the driver guarantees it).
  // Advances `state` and returns the decomposed terms; sum them with
  // PidTerms::output().
  PidTerms<Scalar, N> update(PidState<Scalar, N>& state,
                             const Eigen::Ref<const Array>& setpoint,
                             const Eigen::Ref<const Array>& meas, Scalar dt,
                             const PidModifiers<Scalar, N>& mods = {}) const {
    // Prime derivative history on the first sample so we never differentiate
    // against a stale (default-zero) measurement.
    if (!state.seeded) {
      state.prev_meas = meas;
      state.d_term.setZero();
      state.seeded = true;
    }

    const Array error = setpoint - meas;

    // Derivative on measurement, not on error: a setpoint step then produces no
    // derivative "kick". The raw difference is low-pass filtered per axis.
    const Array raw_deriv = (meas - state.prev_meas) / dt;
    d_lpf_.apply(state.d_term, raw_deriv, dt);
    state.prev_meas = meas;

    // Integral with three stacked anti-windup mechanisms:
    //   1. soft authority taper as |error| grows (PX4 i_factor),
    //   2. conditional integration against a saturated output,
    //   3. a hard clamp on the accumulated state.
    const Array i_factor =
        (Scalar(1) - (error / cfg_.i_decay_error).square()).max(Scalar(0));
    Array i_err = error;
    i_err = mods.sat_hi.select(i_err.min(Scalar(0)), i_err);
    i_err = mods.sat_lo.select(i_err.max(Scalar(0)), i_err);
    state.integrator += mods.i_scale * cfg_.ki * i_factor * i_err * dt;
    state.integrator = state.integrator.max(-cfg_.i_max).min(cfg_.i_max);

    PidTerms<Scalar, N> terms;
    terms.p = mods.pd_scale * cfg_.kp * error;
    terms.i = state.integrator;
    // d/dt(error) = -d/dt(meas) for a fixed setpoint, hence the minus sign.
    terms.d = mods.pd_scale * cfg_.kd * (-state.d_term);
    terms.ff = cfg_.kff * setpoint;
    return terms;
  }

 private:
  PidCfg<Scalar, N> cfg_;
  Pt1Filter<Scalar, N> d_lpf_;
};

}  // namespace apl

#endif  // APL_PID_HPP_
