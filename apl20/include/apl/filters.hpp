#ifndef APL_FILTERS_HPP_
#define APL_FILTERS_HPP_

#include <Eigen/Core>
#include <numbers>

namespace apl {

// First-order (PT1) low-pass, vectorized over Dim axes and driven by the
// timestep supplied to `apply` rather than a baked-in sample rate: k =
// dt/(dt+RC) with RC = 1/(2*pi*fc) is recomputed every call, so the filter
// stays correct under a variable loop period (ArduPilot's LowPassFilter / PX4's
// AlphaFilter).
//
// Stateless in the apl sense -- it holds only its cutoff (config) and `apply`
// is const, advancing the caller's `state` in place. The state therefore lives
// in the owning controller's state struct, right beside the rest of its mutable
// data. A per-axis cutoff of +inf passes that axis straight through (k -> 1).
template <typename Scalar, int Dim>
class Pt1Filter {
 public:
  using State = Eigen::Array<Scalar, Dim, 1>;

  template <typename Derived>
  explicit Pt1Filter(const Eigen::ArrayBase<Derived>& cutoff_hz)
      : cutoff_hz_(cutoff_hz) {}
  explicit Pt1Filter(Scalar cutoff_hz)
      : cutoff_hz_(State::Constant(cutoff_hz)) {}

  // Advance `state` one sample: state += dt/(dt+RC) * (input - state).
  template <typename Derived>
  void apply(State& state, const Eigen::ArrayBase<Derived>& input,
             Scalar dt) const {
    constexpr Scalar kInvTwoPi = Scalar(0.5) / std::numbers::pi_v<Scalar>;
    const State rc = kInvTwoPi / cutoff_hz_;  // RC = 1/(2*pi*fc), per axis
    state += (dt / (dt + rc)) * (input - state);
  }

 private:
  State cutoff_hz_;
};

// Per-axis acceleration (slew-rate) limiter, in the same stateless idiom as
// Pt1Filter: holds only the per-axis limit (config), `apply` is const and steps
// the caller's `state` toward `desired_rate` by at most accel_max*dt. Passing
// dt per call keeps the per-second limit correct under a variable loop period.
// A non-positive limit disables that axis (the desired value passes through).
template <typename Scalar, int Dim>
class SlewRateLimiter {
 public:
  using State = Eigen::Array<Scalar, Dim, 1>;

  template <typename Derived>
  explicit SlewRateLimiter(const Eigen::ArrayBase<Derived>& accel_max)
      : accel_max_(accel_max) {}
  explicit SlewRateLimiter(Scalar accel_max)
      : accel_max_(State::Constant(accel_max)) {}

  template <typename Derived>
  void apply(State& state, const Eigen::ArrayBase<Derived>& desired_rate,
             Scalar dt) const {
    const State delta = accel_max_ * dt;
    const State clamped = desired_rate.max(state - delta).min(state + delta);
    state = (accel_max_ > Scalar(0)).select(clamped, desired_rate);
  }

  const State& accel_max() const { return accel_max_; }

 private:
  State accel_max_;
};

}  // namespace apl

#endif  // APL_FILTERS_HPP_
