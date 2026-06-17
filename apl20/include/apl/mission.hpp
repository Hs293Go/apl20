#ifndef APL_MISSION_HPP_
#define APL_MISSION_HPP_

#include <Eigen/Core>
#include <cmath>
#include <concepts>
#include <vector>

namespace apl {

// A mission waypoint: an absolute pose target -- ENU position + heading -- for
// the position controller to navigate to in sequence.
template <std::floating_point Scalar>
struct Waypoint {
  Eigen::Vector3<Scalar> position;  // absolute ENU [m]
  Scalar yaw = Scalar(0);           // absolute heading [rad]
};

// Square pattern (ENU): the four corners of an axis-aligned square of side
// `side` centred at `center`, traversed counter-clockwise (viewed from above)
// and closed back to the first corner (5 waypoints). All hold heading `yaw`.
template <std::floating_point Scalar>
std::vector<Waypoint<Scalar>> SquarePattern(
    const Eigen::Vector3<Scalar>& center, Scalar side, Scalar yaw) {
  const Scalar h = side / Scalar(2);
  const Scalar cx = center.x();  // east
  const Scalar cy = center.y();  // north
  const Scalar cz = center.z();  // up (altitude)
  return {
      {Eigen::Vector3<Scalar>(cx - h, cy - h, cz), yaw},  // SW
      {Eigen::Vector3<Scalar>(cx + h, cy - h, cz), yaw},  // SE
      {Eigen::Vector3<Scalar>(cx + h, cy + h, cz), yaw},  // NE
      {Eigen::Vector3<Scalar>(cx - h, cy + h, cz), yaw},  // NW
      {Eigen::Vector3<Scalar>(cx - h, cy - h, cz), yaw},  // close the loop
  };
}

// Lawnmower / boustrophedon survey (ENU): parallel lanes of length `length`
// (along +x / east from `origin`) covering `width` to the north (+y), lanes
// `spacing` apart, alternating direction so the path is continuous. Heading
// held at `yaw`. The last lane is clamped to the far edge so coverage is exact.
template <std::floating_point Scalar>
std::vector<Waypoint<Scalar>> LawnmowerPattern(
    const Eigen::Vector3<Scalar>& origin, Scalar length, Scalar width,
    Scalar spacing, Scalar yaw) {
  using std::ceil;
  using std::min;
  std::vector<Waypoint<Scalar>> wps;
  const Scalar z = origin.z();  // up (altitude), held across the survey
  const int lanes = static_cast<int>(ceil(width / spacing)) + 1;
  wps.reserve(static_cast<std::size_t>(2 * lanes));
  for (int i = 0; i < lanes; ++i) {
    const Scalar cross =
        origin.y() + min(static_cast<Scalar>(i) * spacing, width);  // north
    const Scalar x_near = origin.x();
    const Scalar x_far = origin.x() + length;
    // Even lanes run near->far, odd lanes far->near (boustrophedon).
    const Scalar a = (i % 2 == 0) ? x_near : x_far;
    const Scalar b = (i % 2 == 0) ? x_far : x_near;
    wps.push_back({Eigen::Vector3<Scalar>(a, cross, z), yaw});
    wps.push_back({Eigen::Vector3<Scalar>(b, cross, z), yaw});
  }
  return wps;
}

}  // namespace apl

#endif  // APL_MISSION_HPP_
