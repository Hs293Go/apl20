#ifndef APL_POSITION_REFERENCE_HPP_
#define APL_POSITION_REFERENCE_HPP_

#include <cmath>

#include "Eigen/Dense"
#include "apl/control_ops.hpp"
#include "apl/filters.hpp"
#include "apl/position_controller.hpp"

namespace apl {

// Tuning for the position trajectory shaper. The per-axis velocity /
// acceleration / jerk limits are what make the reference "achievable": a step
// in the target pose is ramped through a jerk-bounded S-curve instead of
// jumping, so the position controller below never sees a discontinuity. NED,
// [m] axes.
struct PositionReferenceCfg {
  double kp_pos = 1.0;  // pos->vel sqrt-controller linear slope [1/s]
  double kp_vel = 2.0;  // vel->accel sqrt-controller linear slope [1/s]
  // Per-axis kinematic limits [m/s, m/s^2, m/s^3].
  Eigen::Vector3d vel_max = {5, 5, 3};
  Eigen::Vector3d accel_max = {5, 5, 4};
  Eigen::Vector3d jerk_max = {10, 10, 8};
  double yaw_rate_max = 0.5;  // [rad/s] heading slew
};

// Position-domain reference shaper: the trajectory-generation layer above the
// PositionController -- the position analog of AttitudeReference. Given a fixed
// target pose it advances an internal "achievable" (pos, vel, accel) state
// toward it through a jerk-limited kinematic S-curve (ArduPilot's
// shape_pos_vel_accel: sqrt controller pos->vel, sqrt controller vel->accel,
// jerk-slew the accel, then integrate accel->vel->pos). This is what turns a
// step setpoint into PX4/ArduPilot's smooth POSCTL motion -- a naive
// Lee/Sreenath/Mellinger tracker fed a step jumps; here the *reference* moves
// smoothly and the controller only ever tracks a feasible trajectory. Hover and
// takeoff are the trivial case: target directly overhead, the state ramps up
// from rest and settles.
//
// Stateful: owns the (pos, vel, accel, yaw) target. Holds its
// SqrtController/SlewRateLimiter kernels (configured once from cfg) like
// AngularInputShaper does for attitude.
class PositionReference {
 public:
  explicit PositionReference(const PositionReferenceCfg& cfg);

  const PositionReferenceCfg& cfg() const { return cfg_; }

  // Seed the target at the current state (call on (re)start / takeoff so the
  // first step emits no velocity or acceleration spike).
  void reset(const Eigen::Ref<const Eigen::Vector3d>& position,
             const Eigen::Ref<const Eigen::Vector3d>& velocity, double yaw);

  // Advance the achievable trajectory one step toward the fixed target pose
  // (`desired_pos` NED, `desired_yaw` heading [rad]) and emit it as a
  // PositionSetpoint (position + velocity/acceleration feedforward + heading)
  // for the PositionController.
  PositionSetpoint update(const Eigen::Ref<const Eigen::Vector3d>& desired_pos,
                          double desired_yaw, double dt);

 private:
  PositionReferenceCfg cfg_;
  SqrtController<double, 3> pos_sqrt_;  // position -> velocity
  SqrtController<double, 3> vel_sqrt_;  // velocity -> acceleration
  SlewRateLimiter<double, 3> accel_slew_;
  Eigen::Vector3d pos_target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_target_ = Eigen::Vector3d::Zero();
  Eigen::Array3d accel_target_ = Eigen::Array3d::Zero();
  double yaw_target_ = 0;
};

}  // namespace apl

#endif  // APL_POSITION_REFERENCE_HPP_
