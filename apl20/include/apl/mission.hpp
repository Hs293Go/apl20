#ifndef APL_MISSION_HPP_
#define APL_MISSION_HPP_

#include <Eigen/Core>
#include <cmath>
#include <concepts>
#include <vector>

namespace apl {

// A mission waypoint: an absolute pose target -- NED position + heading -- for
// the position controller to navigate to in sequence.
template <std::floating_point Scalar>
struct Waypoint {
  Eigen::Vector3<Scalar> position;  // absolute NED [m]
  Scalar yaw = Scalar(0);           // absolute heading [rad]
};

// Square pattern (NED): the four corners of an axis-aligned square of side
// `side` centred at `center`, traversed counter-clockwise and closed back to
// the first corner (5 waypoints). All hold heading `yaw`.
template <std::floating_point Scalar>
std::vector<Waypoint<Scalar>> SquarePattern(
    const Eigen::Vector3<Scalar>& center, Scalar side, Scalar yaw) {
  const Scalar h = side / Scalar(2);
  const Scalar n = center.x();  // north
  const Scalar e = center.y();  // east
  const Scalar d = center.z();  // down
  return {
      {Eigen::Vector3<Scalar>(n - h, e - h, d), yaw},
      {Eigen::Vector3<Scalar>(n + h, e - h, d), yaw},
      {Eigen::Vector3<Scalar>(n + h, e + h, d), yaw},
      {Eigen::Vector3<Scalar>(n - h, e + h, d), yaw},
      {Eigen::Vector3<Scalar>(n - h, e - h, d), yaw},  // close the loop
  };
}

// Lawnmower / boustrophedon survey (NED): parallel north-south lanes of length
// `length` (along +north from `origin`) covering `width` to the east, lanes
// `spacing` apart, alternating direction so the path is continuous. Heading
// held at `yaw`. The last lane is clamped to the far edge so coverage is exact.
template <std::floating_point Scalar>
std::vector<Waypoint<Scalar>> LawnmowerPattern(
    const Eigen::Vector3<Scalar>& origin, Scalar length, Scalar width,
    Scalar spacing, Scalar yaw) {
  using std::ceil;
  using std::min;
  std::vector<Waypoint<Scalar>> wps;
  const Scalar d = origin.z();
  const int lanes = static_cast<int>(ceil(width / spacing)) + 1;
  wps.reserve(static_cast<std::size_t>(2 * lanes));
  for (int i = 0; i < lanes; ++i) {
    const Scalar e = origin.y() + min(static_cast<Scalar>(i) * spacing, width);
    const Scalar x_near = origin.x();
    const Scalar x_far = origin.x() + length;
    // Even lanes run near->far, odd lanes far->near (boustrophedon).
    const Scalar a = (i % 2 == 0) ? x_near : x_far;
    const Scalar b = (i % 2 == 0) ? x_far : x_near;
    wps.push_back({Eigen::Vector3<Scalar>(a, e, d), yaw});
    wps.push_back({Eigen::Vector3<Scalar>(b, e, d), yaw});
  }
  return wps;
}

}  // namespace apl

#endif  // APL_MISSION_HPP_
