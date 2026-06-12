// GoogleTest tests for the dt-driven filter kernels. Eigen-only. The point of
// the tests is the variable-rate behaviour: the PT1 reaches the same fraction
// of a step after one time constant regardless of how dt subdivides it, and the
// slew step scales with dt. Both filters are stateless -- they hold only their
// config (cutoff / accel limit) and advance a caller-owned state -- so each
// test constructs the filter once and threads its own state array through
// `apply`.
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <numbers>

#include "apl/filters.hpp"

using apl::Pt1Filter;
using apl::SlewRateLimiter;

namespace {

// fc -> inf passes the input through unchanged (k = dt/(dt+0) = 1).
TEST(Pt1Filter, Passthrough) {
  const double inf = std::numeric_limits<double>::infinity();
  const Pt1Filter<double, 3> f(Eigen::Array<double, 3, 1>::Constant(inf));
  Eigen::Array<double, 3, 1> state = Eigen::Array<double, 3, 1>::Zero();
  f.apply(state, Eigen::Array<double, 3, 1>(1.0, -2.0, 3.0), 0.01);
  EXPECT_NEAR(state[0], 1.0, 1e-9) << "pt1: fc=inf passes through";
  EXPECT_NEAR(state[1], -2.0, 1e-9) << "pt1: fc=inf passes through";
  EXPECT_NEAR(state[2], 3.0, 1e-9) << "pt1: fc=inf passes through";
}

// A unit step reaches 1 - 1/e ~= 0.632 after one time constant RC, and that
// fraction is (nearly) independent of how dt subdivides the interval -- the
// whole reason the filter takes dt per call. One stateless filter, a fresh
// state per dt.
TEST(Pt1Filter, TimeConstant) {
  const double rc = 0.1;
  const double fc = 1.0 / (2.0 * std::numbers::pi_v<double> * rc);
  const Pt1Filter<double, 3> f(Eigen::Array<double, 3, 1>::Constant(fc));
  const double target = 1.0 - std::exp(-1.0);
  for (double dt : {0.001, 0.002, 0.005}) {
    Eigen::Array<double, 3, 1> y = Eigen::Array<double, 3, 1>::Zero();
    for (int i = 0; i < static_cast<int>(rc / dt); ++i) {
      f.apply(y, Eigen::Array<double, 3, 1>::Ones(), dt);
    }
    EXPECT_NEAR(y[0], target, 0.015)
        << "pt1: step reaches ~63% at t=RC, independent of dt";
  }
}

// Per-axis cutoffs filter at different rates: a higher cutoff converges faster.
TEST(Pt1Filter, PerAxis) {
  const Pt1Filter<double, 3> f(Eigen::Array<double, 3, 1>(10.0, 1.0, 0.1));
  Eigen::Array<double, 3, 1> y = Eigen::Array<double, 3, 1>::Zero();
  for (int i = 0; i < 20; ++i) {
    f.apply(y, Eigen::Array<double, 3, 1>::Ones(), 0.01);
  }
  EXPECT_TRUE(y[0] > y[1] && y[1] > y[2])
      << "pt1: higher cutoff converges faster, per axis";
}

// The slew steps toward the target by at most accel_max*dt; a larger dt permits
// a proportionally larger step (the per-second limit holds under variable
// rate).
TEST(SlewRateLimiter, RateLimit) {
  const SlewRateLimiter<double, 3> s(Eigen::Array<double, 3, 1>::Constant(1.0));
  Eigen::Array<double, 3, 1> state = Eigen::Array<double, 3, 1>::Zero();
  s.apply(state, Eigen::Array<double, 3, 1>::Constant(10.0), 0.1);
  EXPECT_NEAR(state[0], 0.1, 1e-9) << "slew: step limited to accel_max*dt";
  Eigen::Array<double, 3, 1> state2 = Eigen::Array<double, 3, 1>::Zero();
  s.apply(state2, Eigen::Array<double, 3, 1>::Constant(10.0), 0.2);
  EXPECT_NEAR(state2[0], 0.2, 1e-9)
      << "slew: larger dt -> larger step (per-second limit)";
}

// A non-positive limit disables that axis (passes the desired value through).
TEST(SlewRateLimiter, DisabledAxis) {
  const SlewRateLimiter<double, 3> s(
      Eigen::Array<double, 3, 1>(1.0, 0.0, -1.0));
  Eigen::Array<double, 3, 1> state = Eigen::Array<double, 3, 1>::Zero();
  s.apply(state, Eigen::Array<double, 3, 1>::Constant(10.0), 0.1);
  EXPECT_NEAR(state[0], 0.1, 1e-9)
      << "slew: limited axis stepped by accel_max*dt";
  EXPECT_NEAR(state[1], 10.0, 1e-9)
      << "slew: non-positive limit passes that axis through";
  EXPECT_NEAR(state[2], 10.0, 1e-9)
      << "slew: non-positive limit passes that axis through";
}

// Repeated application ramps to the target at the limit and holds.
TEST(SlewRateLimiter, Converges) {
  const SlewRateLimiter<double, 3> s(Eigen::Array<double, 3, 1>::Constant(2.0));
  Eigen::Array<double, 3, 1> a = Eigen::Array<double, 3, 1>::Zero();
  for (int i = 0; i < 100; ++i) {  // 10 s at 2 unit/s reaches 1
    s.apply(a, Eigen::Array<double, 3, 1>::Constant(1.0), 0.1);
  }
  EXPECT_NEAR(a[0], 1.0, 1e-9) << "slew: ramps to and holds the target";
  EXPECT_NEAR(a[1], 1.0, 1e-9) << "slew: ramps to and holds the target";
  EXPECT_NEAR(a[2], 1.0, 1e-9) << "slew: ramps to and holds the target";
}

}  // namespace
