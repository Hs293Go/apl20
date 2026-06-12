// GoogleTest assertions for the Pid kernel and RateController driver.
#include <gtest/gtest.h>

#include <cmath>

#include "apl/pid.hpp"
#include "apl/rate_controller.hpp"

using apl::Pid;
using apl::PidCfg;
using apl::PidModifiers;
using apl::PidState;
using apl::PidTerms;
using apl::RateController;
using apl::RateControllerCfg;

namespace {

Eigen::Array<double, 1, 1> Scalar(double v) {
  Eigen::Array<double, 1, 1> a;
  a << v;
  return a;
}

// Output is kp * error when only P is set.
TEST(Pid, Proportional) {
  PidCfg<double> cfg;
  cfg.kp << 2.0;
  Pid<double> pid(cfg);
  PidState<double> st;
  PidTerms<double> t = pid.update(st, Scalar(1.0), Scalar(0.0), 0.01);
  EXPECT_NEAR(t.p(0), 2.0, 1e-9) << "P term == kp*error";
  EXPECT_NEAR(t.output()(0), 2.0, 1e-9) << "output == P with only kp";
}

// Integrator ramps up then saturates at i_max.
TEST(Pid, IntegratorClamp) {
  PidCfg<double> cfg;
  cfg.ki << 10.0;
  cfg.i_max << 1.0;
  Pid<double> pid(cfg);
  PidState<double> st;
  PidTerms<double> t;
  for (int k = 0; k < 5; ++k) {
    t = pid.update(st, Scalar(1.0), Scalar(0.0), 0.1);
  }
  EXPECT_NEAR(t.i(0), 1.0, 1e-9) << "integrator clamps to i_max";
}

// With the output flagged saturated-high, a positive error must not grow I.
TEST(Pid, ConditionalAntiWindup) {
  PidCfg<double> cfg;
  cfg.ki << 10.0;
  Pid<double> pid(cfg);
  PidState<double> st;
  PidModifiers<double> mods;
  mods.sat_hi << true;
  PidTerms<double> t = pid.update(st, Scalar(1.0), Scalar(0.0), 0.1, mods);
  EXPECT_NEAR(t.i(0), 0.0, 1e-9) << "sat_hi freezes positive integration";
}

// D acts on measurement: rising measurement => negative D; a setpoint step with
// constant measurement produces no derivative kick.
TEST(Pid, DerivativeOnMeasurement) {
  PidCfg<double> cfg;
  cfg.kd << 1.0;  // d_lpf_hz defaults to inf => no filtering
  Pid<double> pid(cfg);

  PidState<double> st;
  pid.update(st, Scalar(0.0), Scalar(0.0), 0.1);  // seed
  PidTerms<double> rising = pid.update(st, Scalar(0.0), Scalar(0.1), 0.1);
  EXPECT_TRUE(rising.d(0) < 0.0) << "rising measurement => negative D";
  EXPECT_NEAR(rising.d(0), -1.0, 1e-9) << "D == -kd*d(meas)/dt";

  PidState<double> st2;
  pid.update(st2, Scalar(0.0), Scalar(0.0), 0.1);  // seed
  PidTerms<double> step = pid.update(st2, Scalar(5.0), Scalar(0.0), 0.1);
  EXPECT_NEAR(step.d(0), 0.0, 1e-9) << "setpoint step => no derivative kick";
}

// Feed-forward is kff * setpoint.
TEST(Pid, FeedForward) {
  PidCfg<double> cfg;
  cfg.kff << 0.5;
  Pid<double> pid(cfg);
  PidState<double> st;
  PidTerms<double> t = pid.update(st, Scalar(4.0), Scalar(0.0), 0.01);
  EXPECT_NEAR(t.ff(0), 2.0, 1e-9) << "FF == kff*setpoint";
}

// TPA halves P at full throttle (breakpoint 0.5, rate 0.5).
TEST(Pid, Tpa) {
  RateControllerCfg cfg;
  cfg.pid.kp.setConstant(1.0);
  cfg.tpa_breakpoint = 0.5;
  cfg.tpa_rate = 0.5;
  cfg.output_limit = 10.0;
  RateController rc(cfg);
  Eigen::Vector3d sp(0.5, 0.0, 0.0);
  Eigen::Vector3d meas(0.0, 0.0, 0.0);
  Eigen::Vector3d lo = rc.update(sp, meas, 0.0, 0.004);
  Eigen::Vector3d hi = rc.update(sp, meas, 1.0, 0.004);
  EXPECT_NEAR(lo.x(), 0.5, 1e-9) << "TPA off at idle throttle";
  EXPECT_NEAR(hi.x(), 0.25, 1e-9) << "TPA halves P at full throttle";
}

// Output clamps to the configured limit.
TEST(Pid, OutputClamp) {
  RateControllerCfg cfg;
  cfg.pid.kp.setConstant(100.0);
  cfg.output_limit = 1.0;
  RateController rc(cfg);
  Eigen::Vector3d sp(1.0, 0.0, 0.0);
  Eigen::Vector3d meas(0.0, 0.0, 0.0);
  Eigen::Vector3d out = rc.update(sp, meas, 0.0, 0.004);
  EXPECT_NEAR(out.x(), 1.0, 1e-9) << "output clamps to output_limit";
}

}  // namespace
