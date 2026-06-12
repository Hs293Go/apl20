#include "apl/attitude_reference.hpp"
namespace apl {
namespace {
auto xy() { return Eigen::seqN(0, Eigen::fix<2>); }

}  // namespace

AttitudeReference::AttitudeReference(const AttitudeReferenceCfg& cfg)
    : cfg_(cfg),
      shaper_(cfg.input_tc, cfg.ang_vel_max.array(),
              cfg.ang_accel_max.array()) {}

void AttitudeReference::reset(const Eigen::Quaterniond& attitude) {
  attitude_target_ = attitude.normalized();
  ang_vel_target_.setZero();
}

AttitudeSetpoint AttitudeReference::update(const Eigen::Quaterniond& desired,
                                           double yaw_rate_ff, double dt) {
  const Eigen::Quaterniond desired_n = desired.normalized();
  if (!cfg_.feedforward) {
    attitude_target_ = desired_n;
    ang_vel_target_.setZero();
    return {.attitude = attitude_target_, .ang_vel_ff = ang_vel_target_};
  }

  // SO(3)-log error from the achievable target to the desired.
  const Eigen::Array3d error =
      QuaternionToAngleAxis(attitude_target_.conjugate() * desired_n).array();

  // Feedforward rate request: the commanded yaw rate on the z axis.
  const Eigen::Array3d rate_ff(0.0, 0.0, yaw_rate_ff);

  // Shape all three axes at once (sqrt controller + feedforward), rate- and
  // acceleration-limited, then advance the target by the shaped rate.
  ang_vel_target_ =
      shaper_.apply(error, ang_vel_target_.array(), rate_ff, dt).matrix();
  attitude_target_ *= AngleAxisToQuaternion(ang_vel_target_ * dt).normalized();

  return {.attitude = attitude_target_, .ang_vel_ff = ang_vel_target_};
}

Eigen::Vector3d CombineAttitudeRate(const Eigen::Vector3d& corrective,
                                    const Eigen::Vector3d& feedforward,
                                    const Eigen::Vector3d& measured_rate,
                                    double thrust_error_angle,
                                    double threshold) {
  Eigen::Vector3d out;
  if (thrust_error_angle > 2 * threshold) {
    // Large tilt error: thrust-axis correction only, hold yaw to the gyro.
    out(xy()) = corrective(xy());
    out.z() = measured_rate.z();
  } else if (thrust_error_angle > threshold) {
    const double s = 1 - (thrust_error_angle - threshold) / threshold;
    out(xy()) = corrective(xy()) + feedforward(xy()) * s;
    out.z() =
        measured_rate.z() * (1 - s) + (corrective.z() + feedforward.z()) * s;
  } else {
    out = corrective + feedforward;
  }
  return out;
}
}  // namespace apl
