// Self-contained assertion tests for the geometric PositionController.
// GoogleTest, Eigen-only. ENU frame throughout (x east, y north, z up); the
// FLU body z axis is the thrust (up) axis.
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "apl/position_controller.hpp"

using apl::PositionController;
using apl::PositionControllerCfg;
using apl::PositionControllerOutput;
using apl::PositionSetpoint;

namespace {

PositionControllerCfg BaseCfg() {
  PositionControllerCfg cfg;
  cfg.kp_pos = Eigen::Vector3d(1.0, 1.0, 1.0);
  cfg.vel.kp.setConstant(3.0);
  cfg.vel.ki.setConstant(2.0);
  cfg.vel.i_max.setConstant(15.0);
  cfg.hover_thrust = 0.5;
  cfg.gravity = 9.81;
  return cfg;
}

// At the setpoint with zero velocity and a matching hover_thrust, the output is
// exactly hover thrust and a level attitude.
TEST(PositionController, LevelHover) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;  // origin, zero yaw
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  EXPECT_NEAR(out.collective_thrust, 0.5, 1e-9)
      << "hover: collective == hover_thrust";
  EXPECT_TRUE(out.attitude_setpoint.angularDistance(
                  Eigen::Quaterniond::Identity()) < 1e-9)
      << "hover: attitude is level";
}

// THE HEADLINE: configure the WRONG hover thrust (0.5 vs a true 0.7) and let
// the velocity integral absorb the error to hold altitude — the robustness a
// PD-only geometric tracker cannot achieve.
TEST(PositionController, MassUncertainty) {
  const double true_hover =
      0.7;  // plant needs 0.7 to hover; controller thinks 0.5
  PositionController pc(BaseCfg());
  Eigen::Vector3d pos(0.0, 0.0, 1.0);  // 1 m altitude (ENU z = +1)
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  pc.reset(vel);
  PositionSetpoint sp;
  sp.position = Eigen::Vector3d(0.0, 0.0, 1.0);  // hold 1 m

  const double dt = 0.01;
  const double g = 9.81;
  PositionControllerOutput out;
  for (int k = 0; k < 4000; ++k) {  // 40 s
    out = pc.update(pos, vel, sp, dt);
    // Plant: collective c maps to ENU vertical accel via the TRUE hover thrust
    // (more thrust -> +z up).
    const double a_z = g * (out.collective_thrust / true_hover - 1.0);
    vel.z() += a_z * dt;
    pos.z() += vel.z() * dt;
  }
  EXPECT_NEAR(pos.z(), 1.0, 0.05) << "mass uncertainty: holds altitude";
  EXPECT_NEAR(out.collective_thrust, true_hover, 0.03)
      << "mass uncertainty: integral converges to TRUE hover thrust";
}

// A north (+y ENU) position error tilts the vehicle so thrust pushes north: the
// thrust axis (body_z, FLU up) gains a +y component, i.e. body_z.y > 0.
TEST(PositionController, NorthTilt) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;
  sp.position = Eigen::Vector3d(0.0, 1.0, 0.0);  // 1 m north
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  const Eigen::Vector3d body_z =
      out.attitude_setpoint * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(body_z.y() > 0.0) << "north error tilts thrust north";
  EXPECT_NEAR(body_z.x(), 0.0, 1e-9) << "north error: no east tilt";
}

// An east (+x ENU) position error tilts the vehicle so thrust pushes east: the
// thrust axis (body_z) gains a +x component, i.e. body_z.x > 0.
TEST(PositionController, EastTilt) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;
  sp.position = Eigen::Vector3d(1.0, 0.0, 0.0);  // 1 m east
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  const Eigen::Vector3d body_z =
      out.attitude_setpoint * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(body_z.x() > 0.0) << "east error tilts thrust east";
  EXPECT_NEAR(body_z.y(), 0.0, 1e-9) << "east error: no north tilt";
}

// Commanding a descent (setpoint below the vehicle) pulls collective below
// hover; commanding a (modest) climb pushes it above -- the z-thrust sign.
TEST(PositionController, DescentAndClimbThrust) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  const Eigen::Vector3d at(0.0, 0.0, 1.0);  // 1 m altitude (ENU z = +1)

  PositionSetpoint descend;
  descend.position = Eigen::Vector3d(0.0, 0.0, 0.0);  // go down to ground
  EXPECT_LT(
      pc.update(at, Eigen::Vector3d::Zero(), descend, 0.01).collective_thrust,
      BaseCfg().hover_thrust)
      << "descent command: collective below hover";

  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint climb;
  climb.position = Eigen::Vector3d(0.0, 0.0, 2.0);  // climb to 2 m
  EXPECT_GT(
      pc.update(at, Eigen::Vector3d::Zero(), climb, 0.01).collective_thrust,
      BaseCfg().hover_thrust)
      << "climb command: collective above hover";
}

// PX4's angle boost: when tilted (no vertical accel demand) the collective is
// scaled by 1/cos(tilt) so the vertical thrust component still equals hover --
// i.e. collective * body_z.z == hover_thrust exactly.
TEST(PositionController, AngleBoost) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;
  sp.position = Eigen::Vector3d(1.0, 0.0, 0.0);  // pure horizontal: az demand 0
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  const Eigen::Vector3d body_z =
      out.attitude_setpoint * Eigen::Vector3d::UnitZ();
  EXPECT_GT(out.collective_thrust, BaseCfg().hover_thrust)
      << "tilt boosts collective above hover";
  EXPECT_NEAR(out.collective_thrust * body_z.z(), BaseCfg().hover_thrust, 1e-9)
      << "angle boost: vertical thrust component held at hover";
}

// At hover with a nonzero yaw setpoint the attitude is a pure heading rotation:
// level (thrust axis vertical) with the body forward axis at the commanded yaw
// (measured CCW from +x east, per REP-103).
TEST(PositionController, YawSetpoint) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;
  sp.yaw = 0.5;
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  const Eigen::Vector3d body_z =
      out.attitude_setpoint * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d body_x =
      out.attitude_setpoint * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(body_z.x(), 0.0, 1e-9) << "yaw only: level (no tilt)";
  EXPECT_NEAR(body_z.y(), 0.0, 1e-9) << "yaw only: level (no tilt)";
  EXPECT_NEAR(std::atan2(body_x.y(), body_x.x()), 0.5, 1e-9)
      << "yaw only: forward axis at commanded heading";
}

// The acceleration feedforward passes straight through to the thrust vector: a
// +x (east) accel feedforward tilts the thrust axis east, like an east error.
TEST(PositionController, AccelFeedforward) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;  // at the setpoint: zero pos/vel error
  sp.acceleration_ff = Eigen::Vector3d(2.0, 0.0, 0.0);
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  const Eigen::Vector3d body_z =
      out.attitude_setpoint * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(body_z.x() > 0.0) << "east accel feedforward tilts thrust east";
}

// A large climb command saturates collective thrust at thrust_max.
TEST(PositionController, ThrustClamp) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;
  sp.position =
      Eigen::Vector3d(0.0, 0.0, 100.0);  // 100 m up: way beyond authority
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  EXPECT_NEAR(out.collective_thrust, 0.9, 1e-9)
      << "climb saturates at thrust_max";
}

// A large horizontal command clamps the tilt to tilt_max.
TEST(PositionController, TiltLimit) {
  PositionController pc(BaseCfg());
  pc.reset(Eigen::Vector3d::Zero());
  PositionSetpoint sp;
  sp.position =
      Eigen::Vector3d(100.0, 0.0, 0.0);  // 100 m east: way beyond authority
  PositionControllerOutput out =
      pc.update(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sp, 0.01);
  const Eigen::Vector3d body_z =
      out.attitude_setpoint * Eigen::Vector3d::UnitZ();
  const double tilt = std::acos(std::clamp(body_z.z(), -1.0, 1.0));
  EXPECT_NEAR(tilt, pc.cfg().tilt_max.value(), 1e-6)
      << "tilt clamps to tilt_max";
}

}  // namespace
