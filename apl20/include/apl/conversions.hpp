#ifndef APL_CONVERSIONS_HPP_
#define APL_CONVERSIONS_HPP_

#include <numbers>

#include "Eigen/Dense"

namespace apl {

template <typename Derived>
Eigen::Vector3<typename Derived::Scalar> InterconvertNedEnu(
    const Eigen::MatrixBase<Derived>& vector) {
  return {vector.y(), vector.x(), -vector.z()};
}

template <typename Derived>
Eigen::Vector3<typename Derived::Scalar> InterconvertFluFrd(
    const Eigen::MatrixBase<Derived>& vector) {
  return {vector.x(), -vector.y(), -vector.z()};
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> InterconvertAeroRos(
    const Eigen::QuaternionBase<Derived>& attitude_body2world) {
  using Scalar = typename Derived::Scalar;

  constexpr Scalar kSqrt2By2 = std::numbers::sqrt2_v<Scalar> / Scalar(2);

  return {-(kSqrt2By2 * (attitude_body2world.z() + attitude_body2world.w())),
          -(kSqrt2By2 * (attitude_body2world.x() + attitude_body2world.y())),
          -(kSqrt2By2 * (attitude_body2world.x() - attitude_body2world.y())),
          (kSqrt2By2 * (attitude_body2world.z() - attitude_body2world.w()))};
}

}  // namespace apl

#endif  // APL_CONVERSIONS_HPP_
