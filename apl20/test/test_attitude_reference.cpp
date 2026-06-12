// Self-contained assertion tests for the attitude input shaper + feedforward
// combine. GoogleTest, Eigen-only. The combine tests explicitly exercise the
// large-thrust-error path that the FSC-Lab port computed and then discarded (it
// returned the un-blended feedback).
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "apl/attitude_reference.hpp"

using apl::AttitudeReference;
using apl::AttitudeReferenceCfg;
using apl::AttitudeSetpoint;
using apl::CombineAttitudeRate;

namespace {

bool Near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) < tol;
}

Eigen::Quaterniond AboutX(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()));
}

// --- vectorized sqrt controller / input shaping ----------------------------

TEST(AttitudeReference, SqrtControllerVectorized) {
  // Three regimes in one element-wise call: linear-then-clamped, sqrt, and
  // sqrt-then-clamped with the sign preserved.
  const Eigen::Array<double, 3, 1> error(0.01, 5.0, -3.0);
  const Eigen::Array<double, 3, 1> lim(100.0, 2.0, 2.0);
  const apl::SqrtController<double, 3> sqrt(
      Eigen::Array<double, 3, 1>::Constant(10.0), lim);
  const Eigen::Array<double, 3, 1> r = sqrt.apply(error, 1.0);
  // axis 0: linear rate 0.1, but clamped to |error|/dt = 0.01.
  EXPECT_NEAR(r[0], 0.01, 1e-9)
      << "sqrt(vec): linear regime clamped to |error|/dt";
  // axis 1: sqrt regime, sqrt(2*lim*(|e| - linear_dist/2)).
  const double lin1 = 2.0 / (10.0 * 10.0);
  EXPECT_NEAR(r[1], std::sqrt(2.0 * 2.0 * (5.0 - lin1 / 2.0)), 1e-9)
      << "sqrt(vec): sqrt regime";
  // axis 2: sqrt rate ~3.46 clamped to |error|/dt = 3, sign preserved.
  EXPECT_NEAR(r[2], -3.0, 1e-9)
      << "sqrt(vec): sqrt regime clamped, sign preserved";
}

TEST(AttitudeReference, InputShapeAngleVectorized) {
  // From rest: error axis is accel-limited on the first step, and a commanded
  // yaw rate (rate_ff on z) is slewed in under its own accel limit.
  const Eigen::Array<double, 3, 1> error(0.5, 0.0, 0.0);
  const Eigen::Array<double, 3, 1> accel(10.0, 10.0, 4.0);
  const Eigen::Array<double, 3, 1> target = Eigen::Array<double, 3, 1>::Zero();
  const Eigen::Array<double, 3, 1> ff(0.0, 0.0, 1.0);  // 1 rad/s commanded yaw
  const Eigen::Array<double, 3, 1> vmax(20.0, 20.0, 20.0);
  const double dt = 0.01;
  const apl::AngularInputShaper<double> shaper(0.15, vmax, accel);
  const Eigen::Array<double, 3, 1> r = shaper.apply(error, target, ff, dt);
  EXPECT_TRUE(r[0] > 0.0 && r[0] <= accel[0] * dt + 1e-12)
      << "inputshape(vec): roll first step accel-limited";
  EXPECT_TRUE(r[2] > 0.0 && r[2] <= accel[2] * dt + 1e-12)
      << "inputshape(vec): yaw feedforward slewed in (accel-limited)";
}

// --- the shaper ------------------------------------------------------------

// The achievable target converges to the desired attitude, and the feedforward
// decays to zero at convergence.
TEST(AttitudeReference, ShaperConverges) {
  AttitudeReferenceCfg cfg;
  AttitudeReference ref(cfg);
  ref.reset(Eigen::Quaterniond::Identity());
  const Eigen::Quaterniond desired = AboutX(0.5);
  AttitudeSetpoint sp;
  for (int k = 0; k < 4000; ++k) {  // 10 s at 400 Hz
    sp = ref.update(desired, 0.0, 0.0025);
  }
  EXPECT_TRUE(sp.attitude.angularDistance(desired) < 1e-4)
      << "shaper: target reaches desired";
  EXPECT_TRUE(sp.ang_vel_ff.norm() < 1e-4)
      << "shaper: feedforward decays to zero";
}

// From rest, the first step's feedforward rate is bounded by accel_max*dt: the
// target is acceleration-limited (achievable), not a step.
TEST(AttitudeReference, ShaperAccelLimited) {
  AttitudeReferenceCfg cfg;
  AttitudeReference ref(cfg);
  ref.reset(Eigen::Quaterniond::Identity());
  const double dt = 0.0025;
  AttitudeSetpoint sp = ref.update(AboutX(1.0), 0.0, dt);  // big error
  EXPECT_TRUE(sp.ang_vel_ff.x() <= cfg.ang_accel_max.x() * dt + 1e-12)
      << "shaper: first-step FF rate <= accel_max*dt";
  EXPECT_TRUE(sp.ang_vel_ff.x() > 0.0) << "shaper: FF rate ramps up (nonzero)";
}

// With feedforward disabled, the target jumps to desired and FF is zero.
TEST(AttitudeReference, ShaperFeedforwardDisabled) {
  AttitudeReferenceCfg cfg;
  cfg.feedforward = false;
  AttitudeReference ref(cfg);
  ref.reset(Eigen::Quaterniond::Identity());
  AttitudeSetpoint sp = ref.update(AboutX(0.5), 0.0, 0.0025);
  EXPECT_TRUE(sp.attitude.angularDistance(AboutX(0.5)) < 1e-9)
      << "FF off: target == desired";
  EXPECT_TRUE(sp.ang_vel_ff.norm() < 1e-12) << "FF off: feedforward is zero";
}

// --- the feedforward combine (the corrected large-error path) --------------

TEST(AttitudeReference, CombineSmallError) {
  const Eigen::Vector3d corrective(1, 2, 3);
  const Eigen::Vector3d ff(10, 20, 30);
  const Eigen::Vector3d gyro(0.1, 0.2, 0.3);
  // thrust error below threshold => full feedforward.
  const Eigen::Vector3d r = CombineAttitudeRate(corrective, ff, gyro, 0.1, 0.5);
  EXPECT_TRUE(Near(r.x(), 11) && Near(r.y(), 22) && Near(r.z(), 33))
      << "combine: small error => corrective + feedforward";
}

TEST(AttitudeReference, CombineLargeError) {
  const Eigen::Vector3d corrective(1, 2, 3);
  const Eigen::Vector3d ff(10, 20, 30);
  const Eigen::Vector3d gyro(0.1, 0.2, 0.3);
  // thrust error past 2x threshold => thrust-axis correction only, yaw held to
  // gyro. (FSC-Lab returned (1,2,3) here -- feedback only, yaw NOT held.)
  const Eigen::Vector3d r = CombineAttitudeRate(corrective, ff, gyro, 1.5, 0.5);
  EXPECT_TRUE(Near(r.x(), 1) && Near(r.y(), 2))
      << "combine: large error => no roll/pitch FF";
  EXPECT_NEAR(r.z(), 0.3, 1e-9)
      << "combine: large error => yaw held to gyro (the dropped path)";
}

TEST(AttitudeReference, CombineMidError) {
  const Eigen::Vector3d corrective(1, 2, 3);
  const Eigen::Vector3d ff(10, 20, 30);
  const Eigen::Vector3d gyro(0.1, 0.2, 0.3);
  // thrust error = 0.75, threshold 0.5 => s = 1 - (0.75-0.5)/0.5 = 0.5.
  const Eigen::Vector3d r =
      CombineAttitudeRate(corrective, ff, gyro, 0.75, 0.5);
  const double s = 0.5;
  EXPECT_TRUE(Near(r.x(), 1 + 10 * s) && Near(r.y(), 2 + 20 * s))
      << "combine: mid error => ramped roll/pitch FF";
  EXPECT_NEAR(r.z(), 0.3 * (1 - s) + (3 + 30) * s, 1e-9)
      << "combine: mid error => blended yaw";
}

}  // namespace
