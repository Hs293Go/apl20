// GoogleTest tests for the mission pattern generators. Eigen-only. NED
// throughout (x north, y east, z down).
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "apl/mission.hpp"

using apl::LawnmowerPattern;
using apl::SquarePattern;
using apl::Waypoint;

namespace {

bool Near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) < tol;
}

bool At(const Waypoint<double>& wp, double n, double e, double d) {
  return Near(wp.position.x(), n) && Near(wp.position.y(), e) &&
         Near(wp.position.z(), d);
}

// Square: 5 waypoints (closed loop) at the corners of a `side` square centred
// at `center`, all holding the commanded heading.
TEST(Mission, Square) {
  const auto wps = SquarePattern(Eigen::Vector3d(0, 0, -2), 4.0, 0.5);
  EXPECT_TRUE(wps.size() == 5) << "square: 5 waypoints (4 corners + close)";
  EXPECT_TRUE(At(wps[0], -2, -2, -2)) << "square: corner 0 (SW)";
  EXPECT_TRUE(At(wps[1], 2, -2, -2)) << "square: corner 1 (SE)";
  EXPECT_TRUE(At(wps[2], 2, 2, -2)) << "square: corner 2 (NE)";
  EXPECT_TRUE(At(wps[3], -2, 2, -2)) << "square: corner 3 (NW)";
  EXPECT_TRUE(At(wps[4], -2, -2, -2)) << "square: closes back to corner 0";
  bool yaw_ok = true;
  for (const auto& wp : wps) {
    yaw_ok = yaw_ok && Near(wp.yaw, 0.5);
  }
  EXPECT_TRUE(yaw_ok) << "square: all hold the commanded heading";
}

// Lawnmower: a 10 x 6 m survey at 3 m spacing => ceil(6/3)+1 = 3 lanes, 6
// waypoints, alternating direction, covering the rectangle exactly.
TEST(Mission, Lawnmower) {
  const auto wps =
      LawnmowerPattern(Eigen::Vector3d(0, 0, -5), 10.0, 6.0, 3.0, 0.0);
  EXPECT_TRUE(wps.size() == 6) << "mow: 3 lanes -> 6 waypoints";
  // Lane 0 near->far at e=0; lane 1 far->near at e=3; lane 2 near->far at e=6.
  EXPECT_TRUE(At(wps[0], 0, 0, -5) && At(wps[1], 10, 0, -5))
      << "mow: lane 0 near->far";
  EXPECT_TRUE(At(wps[2], 10, 3, -5) && At(wps[3], 0, 3, -5))
      << "mow: lane 1 far->near";
  EXPECT_TRUE(At(wps[4], 0, 6, -5) && At(wps[5], 10, 6, -5))
      << "mow: lane 2 near->far";
}

// Lawnmower coverage clamps the last lane to the far edge when `width` is not a
// multiple of `spacing` (width 5, spacing 3 -> lanes at e = 0, 3, 5).
TEST(Mission, LawnmowerClamp) {
  const auto wps =
      LawnmowerPattern(Eigen::Vector3d(0, 0, 0), 8.0, 5.0, 3.0, 0.0);
  EXPECT_TRUE(wps.size() == 6) << "mow-clamp: 3 lanes";
  EXPECT_TRUE(Near(wps[4].position.y(), 5.0) && Near(wps[5].position.y(), 5.0))
      << "mow-clamp: last lane clamped to width (e = 5, not 6)";
  // Continuity: each lane starts where the previous ended in north.
  bool continuous = true;
  for (std::size_t i = 1; i < wps.size(); i += 2) {
    if (i + 1 < wps.size()) {
      continuous =
          continuous && Near(wps[i].position.x(), wps[i + 1].position.x());
    }
  }
  EXPECT_TRUE(continuous) << "mow-clamp: lane transitions are cross-track only";
}

}  // namespace
