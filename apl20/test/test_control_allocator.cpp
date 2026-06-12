// Self-contained assertion tests for the control allocation kernel. GoogleTest
// executable, Eigen-only. Built against the PX4 SITL x500 quad geometry
// (airframe 4001_gz_x500, CA_ROTOR{0..3}_{PX,PY,KM}) so the mix and its
// desaturation are exercised on the same frame the SITL hover flies.
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <array>

#include "apl/control_allocator.hpp"

using apl::ControlAllocator;
using apl::MakeControlAllocatorCfg;
using apl::Rotor;

namespace {

// x500 SITL geometry: one rotor per motor. Motors 0,1 spin CCW (km > 0), 2,3 CW
// (km < 0); 0,2 front (arm_x > 0), 1,3 rear.
std::array<Rotor<double>, 4> X500Geometry() {
  return {{
      {.arm_x = 0.13, .arm_y = 0.22, .km = 0.05},    // 0: front-left,  CCW
      {.arm_x = -0.13, .arm_y = -0.20, .km = 0.05},  // 1: rear-right,  CCW
      {.arm_x = 0.13, .arm_y = -0.22, .km = -0.05},  // 2: front-right, CW
      {.arm_x = -0.13, .arm_y = 0.20, .km = -0.05},  // 3: rear-left,   CW
  }};
}

ControlAllocator<double, 4> MakeX500Allocator() {
  return ControlAllocator<double, 4>(MakeControlAllocatorCfg(X500Geometry()));
}

// Pure collective spreads evenly: collective [0,1] passes straight through to
// every motor (the normalization makes the collective column all-ones).
TEST(ControlAllocator, PureCollective) {
  const ControlAllocator<double, 4> alloc = MakeX500Allocator();
  const Eigen::Matrix<double, 4, 1> m =
      alloc.allocate(Eigen::Vector3d::Zero(), 0.5);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(m[i], 0.5, 1e-9) << "collective: each motor == collective";
  }
}

// Pure roll: differential thrust with no change in mean (collective preserved),
// and the right sign pattern for the geometry (motors with arm_y > 0 drop).
TEST(ControlAllocator, PureRoll) {
  const ControlAllocator<double, 4> alloc = MakeX500Allocator();
  const Eigen::Matrix<double, 4, 1> m =
      alloc.allocate(Eigen::Vector3d(0.1, 0.0, 0.0), 0.5);
  EXPECT_NEAR(m.mean(), 0.5, 1e-9) << "roll: mean (collective) preserved";
  EXPECT_TRUE(m[0] < 0.5 && m[3] < 0.5 && m[1] > 0.5 && m[2] > 0.5)
      << "roll: +roll lifts arm_y<0 motors, drops arm_y>0 motors";
}

// Pure yaw: reaction-torque differential between the two spin groups, mean
// preserved.
TEST(ControlAllocator, PureYaw) {
  const ControlAllocator<double, 4> alloc = MakeX500Allocator();
  const Eigen::Matrix<double, 4, 1> m =
      alloc.allocate(Eigen::Vector3d(0.0, 0.0, 0.1), 0.5);
  EXPECT_NEAR(m.mean(), 0.5, 1e-9) << "yaw: mean (collective) preserved";
  EXPECT_TRUE(m[0] > 0.5 && m[1] > 0.5 && m[2] < 0.5 && m[3] < 0.5)
      << "yaw: +yaw lifts the km>0 group, drops the km<0 group";
}

// Torque and collective are decoupled: the same torque produces the same
// per-motor differential regardless of the collective level (no saturation).
TEST(ControlAllocator, TorqueCollectiveDecoupled) {
  const ControlAllocator<double, 4> alloc = MakeX500Allocator();
  const Eigen::Vector3d tq(0.05, 0.03, 0.02);
  const Eigen::Matrix<double, 4, 1> a = alloc.allocate(tq, 0.4);
  const Eigen::Matrix<double, 4, 1> b = alloc.allocate(tq, 0.6);
  const Eigen::Matrix<double, 4, 1> da =
      a - Eigen::Matrix<double, 4, 1>::Constant(a.mean());
  const Eigen::Matrix<double, 4, 1> db =
      b - Eigen::Matrix<double, 4, 1>::Constant(b.mean());
  EXPECT_TRUE((da - db).cwiseAbs().maxCoeff() < 1e-9)
      << "decoupled: differential independent of collective";
}

// Airmode desaturation: a roll demand that would push a motor past 1.0 is
// absorbed by shifting ALL motors down equally -- the attitude differential is
// kept intact (no net torque from a uniform shift) and only thrust is given up.
TEST(ControlAllocator, AirmodeDesaturation) {
  const ControlAllocator<double, 4> alloc = MakeX500Allocator();
  const Eigen::Vector3d tq(0.2, 0.0, 0.0);
  const double collective = 0.92;

  // Raw (pre-desaturation) mix, recomputed from the kernel's own matrix.
  Eigen::Vector4d w;
  w << tq, collective;
  const Eigen::Matrix<double, 4, 1> raw = alloc.cfg().mix * w;
  EXPECT_TRUE(raw.maxCoeff() > 1.0)
      << "airmode: setup actually saturates a motor";

  const Eigen::Matrix<double, 4, 1> m = alloc.allocate(tq, collective);
  EXPECT_NEAR(m.maxCoeff(), 1.0, 1e-9)
      << "airmode: top motor pulled to the rail";
  EXPECT_TRUE(m.minCoeff() > 0.0)
      << "airmode: bottom motor stays above idle (no clamp)";
  // Equal downward shift => the attitude differential is identical to raw.
  const Eigen::Matrix<double, 4, 1> expected =
      raw.array() - (raw.maxCoeff() - 1.0);
  EXPECT_TRUE((m - expected).cwiseAbs().maxCoeff() < 1e-9)
      << "airmode: result is raw shifted, torque differential preserved";
  EXPECT_TRUE(m.mean() < collective)
      << "airmode: thrust sacrificed, not attitude";
}

// Extreme demand clamps cleanly: every motor stays within [0,1].
TEST(ControlAllocator, ClampsToRange) {
  const ControlAllocator<double, 4> alloc = MakeX500Allocator();
  const Eigen::Matrix<double, 4, 1> m =
      alloc.allocate(Eigen::Vector3d(5.0, -4.0, 3.0), 0.5);
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(m[i] >= 0.0 && m[i] <= 1.0) << "clamp: motor within [0,1]";
  }
}

}  // namespace
