#ifndef APL_ATTITUDE_CONTROLLER_HPP_
#define APL_ATTITUDE_CONTROLLER_HPP_

#include <cmath>
#include <limits>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "apl/math.hpp"

namespace apl {

// Tuning for the tilt-prioritized attitude controller. "Bag of data",
// designated-initialized; defaults to a safe no-op (zero gains, no rate limit).
struct AttitudeControllerCfg {
  // Proportional gains [roll, pitch, yaw]: attitude error [rad] to body-rate
  // setpoint [rad/s]. Yaw is deprioritized simply by a lower kp.z -- the log
  // error decouples yaw from the thrust-axis (tilt) correction, so PX4's
  // separate yaw-weight knob is unnecessary.
  Eigen::Vector3d kp = Eigen::Vector3d::Zero();

  // Per-axis clamp on the body-rate setpoint [rad/s]. Default unbounded.
  Eigen::Vector3d rate_limit =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
};

// Tilt-prioritized quaternion attitude controller (Brescianini), the outer loop
// of the cascade: it maps an attitude error to a body-rate setpoint.
//
// Stateless -- holds only its Cfg, and update() is pure and const. The error is
// the SO(3) log of the tilt-prioritized error quaternion (a true angle * axis,
// linear in the angle), not the half-angle-sine vector 2 * vec(qe). The reduced
// (thrust-axis) attitude is corrected with priority and is never compromised by
// the yaw, which is read independently as the true heading angle.
//
// References: Brescianini, Hehn, D'Andrea, "Nonlinear Quadrocopter Attitude
// Control" (2013); Brescianini, D'Andrea, "Tilt-Prioritized Quadrocopter
// Attitude Control" (2020).
class AttitudeController {
 public:
  explicit AttitudeController(const AttitudeControllerCfg& cfg);

  const AttitudeControllerCfg& cfg() const { return cfg_; }

  // Body-rate setpoint [rad/s] from the current attitude `q` and desired
  // attitude `qd` (both body->world unit quaternions). `yaw_rate_ff` is a
  // world-frame yaw-rate feedforward [rad/s]; pass 0 for none.
  Eigen::Vector3d update(const Eigen::Quaterniond& q,
                         const Eigen::Quaterniond& qd,
                         double yaw_rate_ff = 0) const;

 private:
  AttitudeControllerCfg cfg_;
};

}  // namespace apl

#endif  // APL_ATTITUDE_CONTROLLER_HPP_
