#ifndef APL_TESTING_MATCHERS_HPP_
#define APL_TESTING_MATCHERS_HPP_

// Shared test support for the apl20 kernel suite: Eigen-aware approximate
// comparisons and the rotation builders that every controller test reaches for.
// Plain GoogleTest (no gmock) -- the comparisons return ::testing::
// AssertionResult, so they read as `EXPECT_TRUE(AllClose(a, b))` and carry a
// useful failure message naming the offending element / distance.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace apl::testing {

// Scalar near-equality. Drop-in for the per-file helper the tests duplicated.
inline bool Near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) < tol;
}

// Element-wise approximate equality for any two Eigen expressions (Matrix or
// Array, any size). On mismatch, reports the first offending element so the
// failure is self-explanatory.
template <typename DerivedA, typename DerivedB>
::testing::AssertionResult AllClose(const Eigen::DenseBase<DerivedA>& a,
                                    const Eigen::DenseBase<DerivedB>& b,
                                    double tol = 1e-6) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return ::testing::AssertionFailure()
           << "shape mismatch: (" << a.rows() << "x" << a.cols() << ") vs ("
           << b.rows() << "x" << b.cols() << ")";
  }
  for (Eigen::Index j = 0; j < a.cols(); ++j) {
    for (Eigen::Index i = 0; i < a.rows(); ++i) {
      const double d = std::abs(static_cast<double>(a.derived().coeff(i, j)) -
                                static_cast<double>(b.derived().coeff(i, j)));
      if (!(d <= tol)) {
        return ::testing::AssertionFailure()
               << "element (" << i << "," << j << ") differs by " << d
               << " > tol " << tol << ": " << a.derived().coeff(i, j) << " vs "
               << b.derived().coeff(i, j);
      }
    }
  }
  return ::testing::AssertionSuccess();
}

// Quaternion equivalence via geodesic distance: 0 for the same rotation,
// double-cover (q and -q) safe. Scalar-generic to match the kernels.
template <typename Scalar>
::testing::AssertionResult QuaternionClose(const Eigen::Quaternion<Scalar>& a,
                                           const Eigen::Quaternion<Scalar>& b,
                                           double tol = 1e-9) {
  const double d = a.angularDistance(b);
  if (std::abs(d) <= tol) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "quaternions differ by " << d << " rad > tol " << tol;
}

// A matrix is orthogonal iff M*Mᵀ and Mᵀ*M are both the identity.
template <typename Derived>
::testing::AssertionResult IsOrthogonal(const Eigen::MatrixBase<Derived>& m,
                                        double tol = 1e-8) {
  if ((m.derived() * m.derived().transpose()).isIdentity(tol) &&
      (m.derived().transpose() * m.derived()).isIdentity(tol)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "matrix is not orthogonal:\n"
                                       << m.derived();
}

// --- rotation builders (unit quaternions) ----------------------------------

inline Eigen::Quaterniond AboutX(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()));
}

inline Eigen::Quaterniond AboutY(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()));
}

inline Eigen::Quaterniond AboutZ(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()));
}

// SO(3) exp map: a rotation vector (angle * axis) to a unit quaternion.
inline Eigen::Quaterniond ExpMap(const Eigen::Vector3d& rotvec) {
  const double a = rotvec.norm();
  if (a < 1e-15) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, rotvec / a));
}

}  // namespace apl::testing

#endif  // APL_TESTING_MATCHERS_HPP_
