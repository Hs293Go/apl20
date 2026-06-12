// Self-contained assertion tests for the tilt-prioritized AttitudeController.
// GoogleTest, Eigen-only. The error is the SO(3) log, so the expected values
// are the TRUE angle errors (kp * angle), not 2*sin(angle/2).
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "apl/attitude_controller.hpp"

using apl::AttitudeController;
using apl::AttitudeControllerCfg;

namespace {

bool Near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) < tol;
}

Eigen::Quaterniond AboutX(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()));
}
Eigen::Quaterniond AboutY(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()));
}
Eigen::Quaterniond AboutZ(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()));
}

// exp map: rotation vector (angle*axis) -> unit quaternion.
Eigen::Quaterniond ExpMap(const Eigen::Vector3d& rotvec) {
  const double a = rotvec.norm();
  if (a < 1e-15) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, rotvec / a));
}

// Zero attitude error => zero rate setpoint.
TEST(AttitudeController, ZeroError) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(6.0, 6.0, 3.0);
  AttitudeController ctrl(cfg);
  Eigen::Vector3d r = ctrl.update(Eigen::Quaterniond::Identity(),
                                  Eigen::Quaterniond::Identity());
  EXPECT_TRUE(r.norm() < 1e-12) << "zero error => zero rate";
}

// A pure roll error gives a pure roll rate of kp.x * (TRUE angle).
TEST(AttitudeController, PureRoll) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(4.0, 5.0, 3.0);
  AttitudeController ctrl(cfg);
  const double theta = 0.6;  // large enough that angle != 2*sin(angle/2)
  Eigen::Vector3d r =
      ctrl.update(Eigen::Quaterniond::Identity(), AboutX(theta));
  EXPECT_NEAR(r.x(), 4.0 * theta, 1e-9) << "roll rate == kp.x * true angle";
  EXPECT_TRUE(Near(r.y(), 0.0) && Near(r.z(), 0.0))
      << "roll error stays on roll axis";
}

// A pure pitch error gives a pure pitch rate of kp.y * (TRUE angle).
TEST(AttitudeController, PurePitch) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(4.0, 5.0, 3.0);
  AttitudeController ctrl(cfg);
  const double theta = 0.5;
  Eigen::Vector3d r =
      ctrl.update(Eigen::Quaterniond::Identity(), AboutY(theta));
  EXPECT_NEAR(r.y(), 5.0 * theta, 1e-9) << "pitch rate == kp.y * true angle";
  EXPECT_TRUE(Near(r.x(), 0.0) && Near(r.z(), 0.0))
      << "pitch error stays on pitch axis";
}

// The headline log win: a LARGE yaw error is linear in the angle, not
// compressed to 2*sin(psi/2). At psi=2.5, the sine form would give 1.90, not
// 2.5.
TEST(AttitudeController, YawLinear) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(4.0, 5.0, 3.0);
  AttitudeController ctrl(cfg);
  const double psi = 2.5;
  Eigen::Vector3d r = ctrl.update(Eigen::Quaterniond::Identity(), AboutZ(psi));
  EXPECT_NEAR(r.z(), 3.0 * psi, 1e-9)
      << "yaw rate == kp.z * true angle (linear)";
  EXPECT_TRUE(Near(r.x(), 0.0) && Near(r.y(), 0.0))
      << "yaw error stays on yaw axis";
}

// World-frame yaw-rate feedforward maps onto the body yaw axis when level.
TEST(AttitudeController, YawFeedforward) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(6.0, 6.0, 3.0);
  AttitudeController ctrl(cfg);
  Eigen::Vector3d r = ctrl.update(Eigen::Quaterniond::Identity(),
                                  Eigen::Quaterniond::Identity(), 0.3);
  EXPECT_TRUE(Near(r.x(), 0.0) && Near(r.y(), 0.0) && Near(r.z(), 0.3))
      << "yaw feedforward maps to body yaw rate";
}

// The rate setpoint is clamped per axis to the configured limit.
TEST(AttitudeController, RateLimit) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(10.0, 10.0, 10.0);
  cfg.rate_limit = Eigen::Vector3d(1.0, 1.0, 1.0);
  AttitudeController ctrl(cfg);
  Eigen::Vector3d r = ctrl.update(Eigen::Quaterniond::Identity(),
                                  AboutX(2.0));  // large roll error
  EXPECT_NEAR(r.x(), 1.0, 1e-9) << "rate setpoint clamps to rate_limit";
}

// A tilted vehicle already at the desired roll, needing only pitch, must get a
// PURE pitch rate (true angle) — no roll/yaw leakage. Regression for the
// body/world frame-consistency bug that q=identity tests miss.
TEST(AttitudeController, TiltedPurePitch) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(4.0, 5.0, 3.0);
  AttitudeController ctrl(cfg);
  const Eigen::Quaterniond q = AboutX(0.5);
  const Eigen::Quaterniond qd = AboutX(0.5) * AboutY(0.3);
  const Eigen::Vector3d r = ctrl.update(q, qd);
  EXPECT_NEAR(r.y(), 5.0 * 0.3, 1e-9)
      << "tilted: pitch error -> pure pitch rate";
  EXPECT_TRUE(Near(r.x(), 0.0) && Near(r.z(), 0.0))
      << "tilted: no spurious roll/yaw";
}

// General check across tilted/coupled attitudes: the error must decompose the
// body error qe into swing (the tilt log, in xy) and twist (the yaw, about z) —
// i.e. exp(eq_xy) * Rz(eq_z) == qe. Independent of the controller's internals,
// and catches any frame inconsistency. Unit gains so rate_sp == eq.
TEST(AttitudeController, Decomposition) {
  AttitudeControllerCfg cfg;
  cfg.kp = Eigen::Vector3d(1.0, 1.0, 1.0);
  AttitudeController ctrl(cfg);
  auto chk = [&](const Eigen::Quaterniond& q, const Eigen::Quaterniond& qd,
                 const char* what) {
    const Eigen::Vector3d eq = ctrl.update(q, qd);
    const Eigen::Quaterniond swing =
        ExpMap(Eigen::Vector3d(eq.x(), eq.y(), 0.0));
    const Eigen::Quaterniond twist(
        Eigen::AngleAxisd(eq.z(), Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond qe_rec = swing * twist;
    const Eigen::Quaterniond qe = q.normalized().conjugate() * qd.normalized();
    EXPECT_TRUE(std::abs(qe_rec.dot(qe)) > 1.0 - 1e-9)
        << what;  // same rotation (+/-)
  };
  chk(Eigen::Quaterniond::Identity(), AboutZ(1.0) * AboutX(0.4),
      "decomp: level, coupled");
  chk(AboutX(0.5), AboutZ(1.5) * AboutY(0.3), "decomp: rolled, coupled");
  chk(AboutZ(0.8) * AboutX(0.3), AboutX(-0.4) * AboutZ(0.6),
      "decomp: general A");
  chk(AboutX(0.9) * AboutY(-0.6) * AboutZ(2.0), AboutX(-0.3) * AboutZ(0.5),
      "decomp: general B");
}

}  // namespace
