// GoogleTest tests for the position trajectory shaper. Eigen-only. Exercises
// the property that matters: a step in the target pose produces a smooth,
// limit-respecting reference (no jump), which is what a naive
// Lee/Sreenath/Mellinger tracker fed a step setpoint lacks.
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "apl/position_reference.hpp"
#include "apl/testing/matchers.hpp"

using apl::PositionReference;
using apl::PositionReferenceCfg;
using apl::PositionSetpoint;
using apl::testing::Near;

namespace {

// Smooth takeoff: from rest on the ground, a step climb command does NOT jump
// -- the first acceleration is jerk-limited (ramps from zero) and the position
// reference barely moves on the first step.
TEST(PositionReference, SmoothTakeoff) {
  PositionReferenceCfg cfg;
  PositionReference ref(cfg);
  ref.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);
  const double dt = 0.004;
  const PositionSetpoint sp = ref.update(Eigen::Vector3d(0, 0, -2), 0.0, dt);
  EXPECT_TRUE(std::abs(sp.acceleration_ff.z()) <= cfg.jerk_max.z() * dt + 1e-9)
      << "takeoff: first accel jerk-limited (ramps from 0)";
  EXPECT_TRUE(std::abs(sp.acceleration_ff.z()) > 0.0)
      << "takeoff: accel starts ramping";
  EXPECT_TRUE(sp.position.norm() < 1e-3)
      << "takeoff: position reference does not jump";
  EXPECT_TRUE(std::abs(sp.velocity_ff.z()) < 0.01)
      << "takeoff: velocity ramps gently";
}

// The achievable state reaches the target pose and settles (vel, accel -> 0).
TEST(PositionReference, Converges) {
  PositionReferenceCfg cfg;
  PositionReference ref(cfg);
  ref.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);
  const double dt = 0.004;
  const Eigen::Vector3d desired(3, -2, -5);
  PositionSetpoint sp;
  for (int k = 0; k < 6000; ++k) {  // 24 s
    sp = ref.update(desired, 1.0, dt);
  }
  EXPECT_TRUE((sp.position - desired).norm() < 1e-2)
      << "converge: reaches target pos";
  EXPECT_TRUE(sp.velocity_ff.norm() < 1e-3)
      << "converge: velocity settles to 0";
  EXPECT_TRUE(sp.acceleration_ff.norm() < 1e-3)
      << "converge: accel settles to 0";
  EXPECT_NEAR(sp.yaw, 1.0, 1e-3) << "converge: yaw reaches target";
}

// Limits respected throughout: |vel| <= vel_max, |accel| <= accel_max, and the
// jerk (|delta accel| / dt) <= jerk_max on every axis, over the whole
// trajectory.
TEST(PositionReference, RespectsLimits) {
  PositionReferenceCfg cfg;
  PositionReference ref(cfg);
  ref.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);
  const double dt = 0.004;
  const Eigen::Vector3d desired(10, 8,
                                -6);  // far target -> saturates vel + accel
  Eigen::Vector3d prev_accel = Eigen::Vector3d::Zero();
  bool vel_ok = true;
  bool accel_ok = true;
  bool jerk_ok = true;
  for (int k = 0; k < 4000; ++k) {
    const PositionSetpoint sp = ref.update(desired, 0.0, dt);
    for (int i = 0; i < 3; ++i) {
      vel_ok = vel_ok && std::abs(sp.velocity_ff[i]) <= cfg.vel_max[i] + 1e-6;
      accel_ok = accel_ok &&
                 std::abs(sp.acceleration_ff[i]) <= cfg.accel_max[i] + 1e-6;
      jerk_ok = jerk_ok && std::abs(sp.acceleration_ff[i] - prev_accel[i]) <=
                               cfg.jerk_max[i] * dt + 1e-9;
    }
    prev_accel = sp.acceleration_ff;
  }
  EXPECT_TRUE(vel_ok) << "limits: velocity within vel_max";
  EXPECT_TRUE(accel_ok) << "limits: acceleration within accel_max";
  EXPECT_TRUE(jerk_ok) << "limits: jerk within jerk_max";
}

// Hover / vertical special case: a target directly overhead produces purely
// vertical motion -- no lateral drift on x or y.
TEST(PositionReference, PureVertical) {
  PositionReferenceCfg cfg;
  PositionReference ref(cfg);
  ref.reset(Eigen::Vector3d(1, 2, 0), Eigen::Vector3d::Zero(), 0.0);
  const double dt = 0.004;
  bool lateral_clean = true;
  for (int k = 0; k < 3000; ++k) {
    const PositionSetpoint sp = ref.update(Eigen::Vector3d(1, 2, -3), 0.0, dt);
    lateral_clean = lateral_clean && Near(sp.position.x(), 1.0, 1e-9) &&
                    Near(sp.position.y(), 2.0, 1e-9);
  }
  EXPECT_TRUE(lateral_clean)
      << "vertical: no lateral drift for an overhead target";
}

// reset seeds the target at the given state: with desired == current, the
// reference stays put (no spurious motion).
TEST(PositionReference, ResetSeeds) {
  PositionReferenceCfg cfg;
  PositionReference ref(cfg);
  ref.reset(Eigen::Vector3d(5, -3, -10), Eigen::Vector3d::Zero(), 0.7);
  const PositionSetpoint sp =
      ref.update(Eigen::Vector3d(5, -3, -10), 0.7, 0.004);
  EXPECT_TRUE((sp.position - Eigen::Vector3d(5, -3, -10)).norm() < 1e-9)
      << "reset: holds the seed pose";
  EXPECT_TRUE(sp.velocity_ff.norm() < 1e-9 && sp.acceleration_ff.norm() < 1e-9)
      << "reset: no motion when already at target";
  EXPECT_NEAR(sp.yaw, 0.7, 1e-9) << "reset: holds the seed heading";
}

// Yaw slews at the rate limit and reaches the commanded heading.
TEST(PositionReference, YawSlew) {
  PositionReferenceCfg cfg;
  PositionReference ref(cfg);
  ref.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);
  const double dt = 0.01;
  PositionSetpoint sp = ref.update(Eigen::Vector3d::Zero(), 1.0, dt);
  EXPECT_TRUE(sp.yaw > 0.0 && sp.yaw <= cfg.yaw_rate_max * dt + 1e-12)
      << "yaw: first step rate-limited";
  for (int k = 0; k < 400; ++k) {  // 4 s > 1.0 / 0.5
    sp = ref.update(Eigen::Vector3d::Zero(), 1.0, dt);
  }
  EXPECT_NEAR(sp.yaw, 1.0, 1e-6) << "yaw: reaches target heading";
}

}  // namespace
