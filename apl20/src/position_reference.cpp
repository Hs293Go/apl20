#include "apl/position_reference.hpp"

#include <algorithm>
#include <numbers>

namespace apl {

PositionReference::PositionReference(const PositionReferenceCfg& cfg)
    : cfg_(cfg),
      pos_sqrt_(Eigen::Array3d::Constant(cfg.kp_pos), cfg.accel_max.array()),
      vel_sqrt_(Eigen::Array3d::Constant(cfg.kp_vel), cfg.jerk_max.array()),
      accel_slew_(cfg.jerk_max.array()) {}

void PositionReference::reset(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& velocity, double yaw) {
  pos_target_ = position;
  vel_target_ = velocity;
  accel_target_.setZero();
  yaw_target_ = yaw;
}

PositionSetpoint PositionReference::update(const Eigen::Vector3d& desired_pos,
                                           double desired_yaw, double dt) {
  // Position error -> velocity setpoint: sqrt controller decelerating at
  // accel_max (finite-time, no overshoot), clamped to vel_max.
  Eigen::Array3d vel_sp =
      pos_sqrt_.apply((desired_pos - pos_target_).array(), dt);
  vel_sp = vel_sp.max(-cfg_.vel_max.array()).min(cfg_.vel_max.array());

  // Velocity error -> acceleration setpoint: sqrt controller decelerating at
  // jerk_max, clamped to accel_max.
  Eigen::Array3d accel_sp = vel_sqrt_.apply(vel_sp - vel_target_.array(), dt);
  accel_sp = accel_sp.max(-cfg_.accel_max.array()).min(cfg_.accel_max.array());

  // Jerk-limit the acceleration, then integrate accel -> vel -> pos.
  accel_slew_.apply(accel_target_, accel_sp, dt);
  vel_target_ += accel_target_.matrix() * dt;
  pos_target_ += vel_target_ * dt;

  // Heading: slew toward the desired yaw at yaw_rate_max, shortest way round.
  using std::clamp;
  using std::remainder;
  constexpr double kTwoPi = 2 * std::numbers::pi;
  const double yaw_err = remainder(desired_yaw - yaw_target_, kTwoPi);
  const double yaw_step = cfg_.yaw_rate_max * dt;
  yaw_target_ =
      remainder(yaw_target_ + clamp(yaw_err, -yaw_step, yaw_step), kTwoPi);

  return {.position = pos_target_,
          .velocity_ff = vel_target_,
          .acceleration_ff = accel_target_.matrix(),
          .yaw = yaw_target_};
}
}  // namespace apl
