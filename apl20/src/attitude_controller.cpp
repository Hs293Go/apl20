#include "apl/attitude_controller.hpp"

namespace apl {

AttitudeController::AttitudeController(const AttitudeControllerCfg& cfg)
    : cfg_(cfg) {}

Eigen::Vector3d AttitudeController::update(const Eigen::Quaterniond& q,
                                           const Eigen::Quaterniond& qd,
                                           double yaw_rate_ff) const {
  using std::atan2;
  using std::copysign;
  using std::fpclassify;
  using std::hypot;
  const Eigen::Quaterniond qn = q.normalized();
  const Eigen::Quaterniond qdn = qd.normalized();

  // Full attitude error in the body frame, split as swing * twist about body
  // z. qt_w = |(qe.w, qe.z)| is the twist magnitude; it vanishes only at a
  // 180 deg tilt (thrust inverted), where the residual yaw is undefined.
  const Eigen::Quaterniond qe = qn.conjugate() * qdn;
  const double qt_w = hypot(qe.w(), qe.z());

  // Error as the SO(3) log (true angle * axis), not 2 * vec(qe). Tilt (xy)
  // and yaw (z) are extracted independently, so the yaw never couples into
  // the thrust-axis correction -- tilt prioritization is structural.
  Eigen::Vector3d eq;
  if (fpclassify(qt_w) == FP_ZERO) {
    // Inverted thrust (180 deg tilt): yaw is undefined; drive the full tilt
    // log. Detect the exact singularity with fpclassify, not a magnitude
    // threshold -- the closed form below is well conditioned for all qt_w >
    // 0.
    eq = QuaternionToAngleAxis(qe);
    eq.z() = 0;
  } else {
    // Reduced (tilt-only) attitude -- the swing. Divide directly (not via a
    // precomputed 1/qt_w, which can overflow for a sub-normal qt_w):
    // qe.{w,z}/qt_w are bounded by 1, so FP_ZERO is the only degenerate
    // input.
    const auto mix_w = qe.w() / qt_w;
    const auto mix_z = qe.z() / qt_w;
    const Eigen::Quaterniond qd_red(
        /*w=*/qt_w,
        /*x=*/mix_w * qe.x() - mix_z * qe.y(),
        /*y=*/mix_w * qe.y() + mix_z * qe.x(),
        /*z=*/0);
    eq = QuaternionToAngleAxis(qd_red);  // tilt rotation vector (z == 0)

    // Residual yaw, read straight off qe as the true angle (atan2 gives the
    // canonical, |.| <= pi heading error directly).
    const auto sign = copysign(1, qe.w());
    eq.z() = 2 * atan2(sign * qe.z(), sign * qe.w());
  }

  // Feed forward the world-frame yaw rate, expressed in the body frame.
  Eigen::Vector3d rate_sp =
      eq.cwiseProduct(cfg_.kp) +
      AttitudeQuaternionBodyZ(qn.conjugate()) * yaw_rate_ff;

  return rate_sp.cwiseMax(-cfg_.rate_limit).cwiseMin(cfg_.rate_limit);
}
}  // namespace apl
