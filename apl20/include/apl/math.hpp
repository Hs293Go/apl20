#ifndef APL_MATH_HPP_
#define APL_MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace apl {

template <typename T>
constexpr T deg2rad(T deg) {
  return deg * std::numbers::pi_v<T> / T(180);
}

template <typename T>
constexpr T rad2deg(T rad) {
  return rad * T(180) / std::numbers::pi_v<T>;
}

// SO(3) logarithm of a unit quaternion: the rotation vector theta * axis,
// whose magnitude is the true rotation angle (wrapped to (-pi, pi]). This is
// Ceres's QuaternionToAngleAxis. Unlike 2 * vec(q) = 2 sin(theta/2) * axis,
// it is linear in the angle, so large attitude errors are not compressed by
// the half-angle sine. Precondition: `quaternion` is (approximately) unit.
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

}  // namespace apl

#endif  // APL_MATH_HPP_
