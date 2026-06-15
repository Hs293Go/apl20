#include "apl/position_controller.hpp"

#include <algorithm>

namespace apl {
namespace {
// Desired attitude from the body-z (thrust down-axis) and a heading.
Eigen::Quaternion<double> attitudeFromBodyZ(const Eigen::Vector3d& body_z,
                                            double yaw) {
  using std::cos;
  using std::sin;
  const Eigen::Vector3d heading(cos(yaw), sin(yaw), 0);
  Eigen::Vector3d body_y = body_z.cross(heading);
  const double body_y_norm = body_y.norm();
  if (body_y_norm > 1e-9) {
    body_y /= body_y_norm;
  } else {
    body_y = Eigen::Vector3d(0, 1,
                             0);  // heading ~ along body z
  }
  const Eigen::Vector3d body_x = body_y.cross(body_z);
  Eigen::Matrix3d rot;
  rot.col(0) = body_x;
  rot.col(1) = body_y;
  rot.col(2) = body_z;
  return Eigen::Quaterniond(rot);
}

}  // namespace
PositionController::PositionController(const PositionControllerCfg& cfg)
    : cfg_(cfg), vel_pid_(cfg.vel) {
  sat_hi_.setZero();
  sat_lo_.setZero();
}

void PositionController::reset(const Eigen::Vector3d& vel) {
  vel_pid_.reset(vel_state_, vel.array());
  sat_hi_.setZero();
  sat_lo_.setZero();
}
PositionControllerOutput PositionController::update(const Eigen::Vector3d& pos,
                                                    const Eigen::Vector3d& vel,
                                                    const PositionSetpoint& sp,
                                                    double dt) {
  using std::cos;
  using std::sin;

  // 1. Position P -> velocity setpoint (+ feedforward).
  const Eigen::Vector3d vel_sp =
      cfg_.kp_pos.cwiseProduct(sp.position - pos) + sp.velocity_ff;

  // 2. Velocity PID -> acceleration. The integral term carries the unknown
  //    hover-thrust / mass bias; conditional anti-windup gates it on thrust
  //    saturation latched from the previous step.
  PidModifiers<double, 3> mods;
  mods.sat_hi = sat_hi_;
  mods.sat_lo = sat_lo_;
  const PidTerms<double, 3> terms =
      vel_pid_.update(vel_state_, vel_sp.array(), vel.array(), dt, mods);
  const Eigen::Vector3d acc_sp = terms.output().matrix() + sp.acceleration_ff;

  // 3. Acceleration -> thrust vector -> (collective, attitude). In NED the
  //    thrust must provide specific force (acc_sp - g); the desired body z
  //    (down axis) points opposite, along (g - acc_sp).
  // Imperative style is the most efficient here: Start by setting body_z to
  // the negated acceleration setpoint, then add gravity and normalize
  Eigen::Vector3d body_z = -acc_sp;
  body_z.z() += cfg_.gravity;
  body_z.normalize();
  limitTilt(body_z);

  // Collective thrust: hover_thrust scaled by the demanded vertical specific
  // force, boosted by 1/cos(tilt) so the vertical component is held while
  // tilted (PX4's angle boost).
  const double thrust_z = acc_sp.z() * (cfg_.hover_thrust / cfg_.gravity) -
                          cfg_.hover_thrust;  // <= 0 (up is -z)
  const double collective_raw = -thrust_z / body_z.z();
  const double collective =
      std::clamp(collective_raw, cfg_.thrust_min, cfg_.thrust_max);

  // Latch thrust saturation for next step's conditional anti-windup on the z
  // velocity integrator: at max thrust the integrator may not demand more
  // climb (more negative acc_z); at min, not more descent.
  sat_hi_ = Mask(false, false, collective <= cfg_.thrust_min);
  sat_lo_ = Mask(false, false, collective >= cfg_.thrust_max);

  PositionControllerOutput out;
  out.attitude_setpoint = attitudeFromBodyZ(body_z, sp.yaw);
  out.collective_thrust = collective;
  out.acceleration_setpoint = acc_sp;
  return out;
}

void PositionController::limitTilt(Eigen::Vector3d& body_z) const {
  using std::cos;
  using std::sin;
  if (cfg_.tilt_max.has_value()) {
    const auto& tilt_max = cfg_.tilt_max.value();
    const double cos_max = cos(tilt_max);
    if (body_z.z() < cos_max) {
      Eigen::Vector3d horizontal(body_z.x(), body_z.y(), 0);
      const double horizontal_norm = horizontal.norm();
      if (horizontal_norm > 1e-9) {
        horizontal *= sin(tilt_max) / horizontal_norm;
      }
      body_z = Eigen::Vector3d(horizontal.x(), horizontal.y(), cos_max);
    }
  }
}
}  // namespace apl
