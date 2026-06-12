#ifndef APL_RATE_CONTROLLER_HPP_
#define APL_RATE_CONTROLLER_HPP_

#include <Eigen/Core>

#include "apl/pid.hpp"

namespace apl {

// Configuration for the body-rate controller: the inner PID tuning plus the
// drone-specific shaping that wraps it. "Bag of data", designated-initialized.
struct RateControllerCfg {
  // Roll / pitch / yaw PID tuning, in that axis order. Gains map body-rate
  // error [rad/s] to normalized torque.
  PidCfg<double, 3> pid;

  // Throttle PID attenuation (Betaflight TPA). Above `tpa_breakpoint` (a
  // normalized throttle in [0, 1]) the P and D gains are bled off linearly,
  // reaching a factor of (1 - tpa_rate) at full throttle. Defaults disable it
  // (breakpoint at 1.0, zero rate).
  double tpa_breakpoint = 1.0;
  double tpa_rate = 0.0;

  // Symmetric clamp on each torque axis, in normalized actuator units. This
  // clamp is also the controller's own saturation signal: an axis that clips
  // here freezes the matching integrator direction next step.
  double output_limit = 1.0;

  // Guard band on the caller-supplied dt [s], rejecting scheduler glitches and
  // startup transients (cf. PX4's [0.125 ms, 20 ms] window).
  double dt_min = 1.0e-4;
  double dt_max = 2.0e-2;
};

// Body-rate controller: the drone-facing driver around the pure Pid kernel.
//
// It owns the kernel's evolving PidState and adds the rate-control concerns the
// kernel deliberately knows nothing about — throttle PID attenuation, output
// normalization, deriving the saturation signal from its own clamp, and dt
// sanitation. Stateless math stays in Pid; this is where the drone logic lives.
class RateController {
 public:
  explicit RateController(const RateControllerCfg& cfg);

  // Run one control step. `rate_sp` and `rate_meas` are body angular rates
  // [rad/s] in (roll, pitch, yaw) order, `throttle` is normalized to [0, 1],
  // and `dt` is the step [s] (clamped internally). Returns the normalized
  // torque setpoint, clamped to the configured output limit.
  Eigen::Vector3d update(const Eigen::Ref<const Eigen::Vector3d>& rate_sp,
                         const Eigen::Ref<const Eigen::Vector3d>& rate_meas,
                         double throttle, double dt);

  // Re-prime the loop from the current measurement: zero the integrator and
  // derivative history and clear latched saturation. Call on arm / mode entry.
  void reset(const Eigen::Ref<const Eigen::Vector3d>& rate_meas);

  // Last step's decomposed P/I/D/FF terms, for logging and tuning telemetry.
  const PidTerms<double, 3>& terms() const { return terms_; }

  const RateControllerCfg& cfg() const { return cfg_; }

 private:
  RateControllerCfg cfg_;
  Pid<double, 3> pid_;
  PidState<double, 3> state_;
  PidTerms<double, 3> terms_;
  Eigen::Array<bool, 3, 1> sat_hi_;  // axes clipped high last step
  Eigen::Array<bool, 3, 1> sat_lo_;  // axes clipped low last step
};

}  // namespace apl

#endif  // APL_RATE_CONTROLLER_HPP_
