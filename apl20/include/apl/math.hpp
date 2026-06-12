#ifndef APL_MATH_HPP_
#define APL_MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>

#include "apl/filters.hpp"

namespace apl {

// SO(3) logarithm of a unit quaternion: the rotation vector theta * axis, whose
// magnitude is the true rotation angle (wrapped to (-pi, pi]). This is Ceres's
// QuaternionToAngleAxis. Unlike 2 * vec(q) = 2 sin(theta/2) * axis, it is
// linear in the angle, so large attitude errors are not compressed by the
// half-angle sine. Precondition: `quaternion` is (approximately) unit.
template <typename Derived>
Eigen::Vector3<typename Derived::Scalar> QuaternionToAngleAxis(
    const Eigen::QuaternionBase<Derived>& quaternion) {
  using Scalar = typename Derived::Scalar;

  using std::atan2;
  using std::copysign;
  using std::fpclassify;
  const Scalar n = quaternion.vec().norm();
  const Scalar& w = quaternion.w();

  if (fpclassify(n) == FP_ZERO) {
    // The actual first taylor term is 2/w, but if the quaternion is normalized
    // and n=0, then w should be 1. Dropping w from the denominator is almost
    // always a valid approximation, and it provides robustness to pathological
    // zero quaternions.
    return Scalar(2) * quaternion.vec();
  }

  // w < 0 ==> cos(theta/2) < 0 ==> theta > pi
  //
  // By convention, the condition |theta| < pi is imposed by wrapping theta
  // to pi; The wrap operation can be folded inside evaluation of atan2
  //
  // theta - pi = atan(sin(theta - pi), cos(theta - pi))
  //            = atan(-sin(theta), -cos(theta))
  const Scalar sign = copysign(Scalar(1), w);
  const Scalar atan_nbyw = atan2(sign * n, sign * w);
  return Scalar(2) * atan_nbyw / n * quaternion.vec();
}

// SO(3) exponential: a rotation vector (angle * axis) -> unit quaternion. The
// inverse of QuaternionToAngleAxis.
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> AngleAxisToQuaternion(
    const Eigen::MatrixBase<Derived>& angle_axis) {
  using Scalar = typename Derived::Scalar;
  using std::cos;
  using std::fpclassify;
  const Scalar angle = angle_axis.norm();
  Eigen::Quaternion<Scalar> q;
  if (fpclassify(angle) == FP_ZERO) {
    q.w() = Scalar(1);
    q.vec() = angle_axis / Scalar(2);
  } else {
    const Scalar half_angle = angle / Scalar(2);
    const Scalar sin_half = std::sin(half_angle);
    q.w() = cos(half_angle);
    q.vec() = sin_half / angle * angle_axis;
  }

  return q;
}

template <typename Derived>
Eigen::Vector3<typename Derived::Scalar> AttitudeQuaternionBodyZ(
    const Eigen::QuaternionBase<Derived>& q) {
  using Scalar = typename Derived::Scalar;
  const Scalar tx = Scalar(2) * q.x();
  const Scalar ty = Scalar(2) * q.y();
  const Scalar tz = Scalar(2) * q.z();
  const Scalar twx = tx * q.w();
  const Scalar twy = ty * q.w();
  const Scalar txx = tx * q.x();
  const Scalar txz = tz * q.x();
  const Scalar tyy = ty * q.y();
  const Scalar tyz = tz * q.y();

  return {txz + twy, tyz - twx, Scalar(1) - (txx + tyy)};
}

// ArduPilot's piecewise P / sqrt controller, element-wise over N axes: the rate
// that drives each `error` to zero such that decelerating at `second_ord_lim`
// (one derivative order above the error) never overshoots -- linear (gain `p`)
// for small error, sqrt for large -- clamped to |error|/dt so it cannot
// overshoot in one step. A non-positive second_ord_lim disables the sqrt regime
// on that axis (stays linear). Precondition: p > 0.
template <typename Scalar, int Dim>
class SqrtController {
 public:
  using Array = Eigen::Array<Scalar, Dim, 1>;
  template <typename Derived1, typename Derived2>
  SqrtController(const Eigen::ArrayBase<Derived1>& p,
                 const Eigen::ArrayBase<Derived2>& second_ord_lim)
      : p_(p), second_ord_lim_(second_ord_lim) {}

  template <typename Derived>
  Eigen::Array<Scalar, Dim, 1> apply(const Eigen::ArrayBase<Derived>& error,
                                     Scalar dt) const {
    const Array abs_err = error.abs();
    // Switchover distance; where there is no acceleration limit it is infinite,
    // so the linear regime is selected everywhere.
    constexpr auto kInf = std::numeric_limits<Scalar>::infinity();
    const Array linear_dist =
        (second_ord_lim_ > Scalar(0))
            .select(second_ord_lim_ / (p_ * p_), Array(kInf));
    const Array sqrt_mag =
        (Scalar(2) * second_ord_lim_ * (abs_err - linear_dist / Scalar(2)))
            .max(Scalar(0))
            .sqrt();
    const Array mag = (abs_err > linear_dist).select(sqrt_mag, p_ * abs_err);
    const Array rate = mag * error.sign();
    const Array bound = abs_err / dt;
    return rate.max(-bound).min(bound);
  }

 private:
  Array p_;
  Array second_ord_lim_;
};

template <std::floating_point Scalar>
class AngularInputShaper {
 public:
  template <typename Derived1, typename Derived2>
  AngularInputShaper(Scalar input_tc,
                     const Eigen::ArrayBase<Derived1>& max_ang_vel,
                     const Eigen::ArrayBase<Derived2>& max_ang_accel)
      : sqrt_controller_(Eigen::Array3<Scalar>::Constant(Scalar(1) / input_tc),
                         max_ang_accel),
        slew_rate_limiter_(max_ang_accel),
        max_ang_vel_(max_ang_vel) {}

  // Input-shape an angle error into an acceleration-limited rate,
  // element-wise (ArduPilot's input_shaping_angle): the sqrt controller
  // gives the rate that converges the error; `rate_ff` is added and the
  // total clamped to `max_rate`, then slewed from the current target rate
  // under `accel_max`. Pass rate_ff = 0 and max_rate <= 0 to disable
  // those.
  template <typename Derived1, typename Derived2, typename Derived3>
  Eigen::Array3<Scalar> apply(const Eigen::ArrayBase<Derived1>& error_angle,
                              const Eigen::ArrayBase<Derived2>& target_rate,
                              const Eigen::ArrayBase<Derived3>& rate_ff,
                              Scalar dt) const {
    Eigen::Array3<Scalar> desired =
        rate_ff + sqrt_controller_.apply(error_angle, dt);
    desired =
        (max_ang_vel_ > Scalar(0))
            .select(desired.max(-max_ang_vel_).min(max_ang_vel_), desired);
    // Slew the current target rate toward `desired` under the accel limit.
    // SlewRateLimiter advances a caller-owned state in place, so copy the
    // incoming target rate in, step it, and return it -- keeping this shaper a
    // pure (target_rate -> rate) function.
    Eigen::Array3<Scalar> rate = target_rate;
    slew_rate_limiter_.apply(rate, desired, dt);
    return rate;
  }

 private:
  SqrtController<Scalar, 3> sqrt_controller_;
  SlewRateLimiter<Scalar, 3> slew_rate_limiter_;
  Eigen::Array3<Scalar> max_ang_vel_;
};

// Angle [rad] between the thrust axes (body z) of two attitudes -- the "tilt
// error" that gates the attitude feedforward.
template <typename Derived1, typename Derived2,
          typename Scalar = typename Derived1::Scalar>
Scalar ThrustErrorAngle(const Eigen::QuaternionBase<Derived1>& q,
                        const Eigen::QuaternionBase<Derived2>& qd) {
  using std::acos;
  const Eigen::Vector3<Scalar> ez = AttitudeQuaternionBodyZ(q);
  const Eigen::Vector3<Scalar> ez_d = AttitudeQuaternionBodyZ(qd);
  return acos(std::clamp(ez.dot(ez_d), Scalar(-1), Scalar(1)));
}

}  // namespace apl

#endif  // APL_MATH_HPP_
