// GoogleTest, Eigen-only. Pins the aerospace<->ROS frame-convention maps used
// at the flight-controller boundary: NED<->ENU and FRD<->FLU vectors, and the
// body->world attitude quaternion reinterpretation. Each map is an involution,
// so the headline property is "apply twice == identity".
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "apl/conversions.hpp"

using apl::InterconvertAeroRos;
using apl::InterconvertFluFrd;
using apl::InterconvertNedEnu;

namespace {

// NED<->ENU: north-east-down (1,2,3) <-> east-north-up (2,1,-3); self-inverse.
TEST(Conversions, NedEnuVector) {
  const Eigen::Vector3d ned(1.0, 2.0, 3.0);
  const Eigen::Vector3d enu = InterconvertNedEnu(ned);
  EXPECT_TRUE(enu.isApprox(Eigen::Vector3d(2.0, 1.0, -3.0)))
      << "ned->enu: swap x/y, negate z";
  EXPECT_TRUE(InterconvertNedEnu(enu).isApprox(ned)) << "ned<->enu: involution";
}

// FRD<->FLU: forward-right-down (1,2,3) <-> forward-left-up (1,-2,-3);
// self-inverse.
TEST(Conversions, FluFrdVector) {
  const Eigen::Vector3d frd(1.0, 2.0, 3.0);
  const Eigen::Vector3d flu = InterconvertFluFrd(frd);
  EXPECT_TRUE(flu.isApprox(Eigen::Vector3d(1.0, -2.0, -3.0)))
      << "frd->flu: negate y, z";
  EXPECT_TRUE(InterconvertFluFrd(flu).isApprox(frd)) << "frd<->flu: involution";
}

// A level, nose-north vehicle is the identity in FRD/NED; in FLU/ENU the same
// physical attitude is a +90 deg yaw (forward = north = +y_enu).
TEST(Conversions, AttitudeIdentityToYaw90) {
  const Eigen::Quaterniond ros =
      InterconvertAeroRos(Eigen::Quaterniond::Identity());
  const Eigen::Quaterniond yaw90(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(ros.angularDistance(yaw90), 0.0, 1e-9)
      << "aero identity -> ros +90 deg yaw";
}

// And the converse: a nose-east vehicle is +90 deg yaw in FRD/NED, identity in
// FLU/ENU (forward = east = +x_enu).
TEST(Conversions, AttitudeYaw90ToIdentity) {
  const Eigen::Quaterniond aero(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond ros = InterconvertAeroRos(aero);
  EXPECT_NEAR(ros.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9)
      << "aero +90 deg yaw -> ros identity";
}

// The attitude map is a self-inverse involution for an arbitrary rotation.
TEST(Conversions, AttitudeInvolution) {
  const Eigen::Quaterniond q =
      Eigen::Quaterniond(
          Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()))
          .normalized();
  const Eigen::Quaterniond back = InterconvertAeroRos(InterconvertAeroRos(q));
  EXPECT_NEAR(back.angularDistance(q), 0.0, 1e-9)
      << "aero<->ros attitude: involution";
}

}  // namespace
