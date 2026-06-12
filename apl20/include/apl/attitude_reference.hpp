#ifndef APL_ATTITUDE_REFERENCE_HPP_
#define APL_ATTITUDE_REFERENCE_HPP_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "apl/math.hpp"

namespace apl {

// Tuning for the attitude input shaper. The accel/rate limits are what make the
// target "achievable" -- the slew respects them, so the rate loop is never
// commanded a rate the airframe cannot reach.
struct AttitudeReferenceCfg {
  double input_tc = 0.15;  // input-shaping time constant [s]
  // Max angular acceleration / velocity per body axis (ArduPilot defaults:
  // ~1100 deg/s^2 roll/pitch, ~270 deg/s^2 yaw).
  Eigen::Vector3d ang_accel_max = {19.2, 19.2, 4.71};
  Eigen::Vector3d ang_vel_max = Eigen::Vector3d::Constant(20);  // [rad/s]
  // Tilt (thrust) error [rad] at which the feedforward starts to be
  // de-prioritized (fully gone by 2x this). 30 deg, per ArduPilot.
  double ff_threshold = 0.5235987755982988;
  bool feedforward = true;
};

// Output of the shaper: the achievable attitude target and the body-rate
// feedforward (the target's angular velocity). The attitude analog of
// PositionSetpoint.
struct AttitudeSetpoint {
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d ang_vel_ff = Eigen::Vector3d::Zero();
};

// ArduPilot-style attitude input shaper: slews an internal "achievable" target
// toward the desired attitude under acceleration/velocity limits, and emits the
// target's angular velocity as feedforward. The reference-generation layer that
// sits above the (stateless) AttitudeController -- the attitude analog of a
// waypoint smoother above the PositionController. Stateful: owns the target.
class AttitudeReference {
 public:
  explicit AttitudeReference(const AttitudeReferenceCfg& cfg);

  const AttitudeReferenceCfg& cfg() const { return cfg_; }

  // Seed the target at the current attitude (call on (re)start so the first
  // step emits no feedforward spike).
  void reset(const Eigen::Quaterniond& attitude);

  // Shape `desired` (the raw attitude setpoint) into an achievable target +
  // feedforward. `yaw_rate_ff` is a commanded world/body yaw rate folded into
  // the yaw shaping; pass 0 for none.
  AttitudeSetpoint update(const Eigen::Quaterniond& desired, double yaw_rate_ff,
                          double dt);

 private:
  AttitudeReferenceCfg cfg_;
  AngularInputShaper<double> shaper_;
  Eigen::Quaterniond attitude_target_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d ang_vel_target_ = Eigen::Vector3d::Zero();
};

// Assemble the body-rate setpoint from the attitude feedback `corrective` and
// the `feedforward`, de-prioritizing the feedforward (and holding yaw to the
// `measured_rate`) as the thrust/tilt error grows past `threshold` --
// ArduPilot's feedforward_scalar. NOTE: the FSC-Lab port computed this blend
// and then returned the un-blended feedback, so the large-error path never ran;
// here the blended result is actually returned.
Eigen::Vector3d CombineAttitudeRate(const Eigen::Vector3d& corrective,
                                    const Eigen::Vector3d& feedforward,
                                    const Eigen::Vector3d& measured_rate,
                                    double thrust_error_angle,
                                    double threshold);

}  // namespace apl

#endif  // APL_ATTITUDE_REFERENCE_HPP_
