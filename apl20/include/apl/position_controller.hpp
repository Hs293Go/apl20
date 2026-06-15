#ifndef APL_POSITION_CONTROLLER_HPP_
#define APL_POSITION_CONTROLLER_HPP_

#include <cmath>
#include <optional>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "apl/math.hpp"
#include "apl/pid.hpp"

namespace apl {

// Reference for the position controller (NED). A bare hover/goto fills only
// `position` and `yaw`; a waypoint/trajectory generator also fills the velocity
// and acceleration feedforwards.
struct PositionSetpoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();     // [m] NED
  Eigen::Vector3d velocity_ff = Eigen::Vector3d::Zero();  // [m/s] feedforward
  Eigen::Vector3d acceleration_ff =
      Eigen::Vector3d::Zero();  // [m/s^2] feedforward
  double yaw = 0.0;             // [rad] heading
};

// Tuning for the geometric position controller.
struct PositionControllerCfg {
  // Position P: position error [m] -> velocity setpoint [m/s].
  Eigen::Vector3d kp_pos = Eigen::Vector3d::Zero();

  // Velocity loop, a PID. Its INTEGRAL term (vel.ki, vel.i_max) is what absorbs
  // mass / hover-thrust error and steady disturbances (wind, battery sag) to
  // hold position with no steady-state offset -- the robustness a naive
  // geometric (PD-only) tracker lacks. Leave vel.kff/vel.kd at zero; use the
  // acceleration feedforward in the setpoint instead.
  PidCfg<double, 3> vel;

  // Normalized collective thrust at hover (acc_z = 0): the operating point the
  // velocity integral works around. Any error in it is corrected by the
  // integral; an online estimator could refine it later.
  double hover_thrust = 0.5;

  double gravity = 9.80665;
  std::optional<double> tilt_max =
      0.7853981633974483;   // max tilt from vertical [rad]
  double thrust_min = 0.1;  // collective thrust clamp [0,1]
  double thrust_max = 0.9;
};

// Decomposed position-controller output: the attitude + collective thrust that
// feed the inner cascade (AttitudeController -> RateController), plus the
// acceleration setpoint for logging / trajectory feedforward.
struct PositionControllerOutput {
  Eigen::Quaterniond attitude_setpoint = Eigen::Quaterniond::Identity();
  double collective_thrust = 0.0;  // normalized [0,1]
  Eigen::Vector3d acceleration_setpoint = Eigen::Vector3d::Zero();
};

// Geometric (Lee/Sreenath) multicopter position controller, made robust to
// mass/thrust uncertainty by a velocity-loop integral -- the mechanism PX4,
// ArduPilot and iNav all rely on. Cascade: position-P -> velocity setpoint;
// velocity-PID (the integral absorbs the unknown hover thrust) -> acceleration
// setpoint; acceleration + gravity -> a thrust vector that splits into a
// collective magnitude and a desired tilt (attitude). The outer loop of the
// stack; its attitude setpoint feeds AttitudeController.
//
// Owns the velocity PID's integrator state, so it is a stateful driver around
// the stateless Pid kernel (like RateController). All quantities are NED.
class PositionController {
 public:
  using Mask = Eigen::Array<bool, 3, 1>;

  explicit PositionController(const PositionControllerCfg& cfg);
  const PositionControllerCfg& cfg() const { return cfg_; }

  // Re-prime the velocity integrator/derivative from the current velocity.
  void reset(const Eigen::Vector3d& vel);

  // One step. `pos`, `vel` are the measured NED position/velocity; `sp` the
  // reference; `dt` the timestep [s]. Returns the thrust + attitude setpoint.
  PositionControllerOutput update(const Eigen::Vector3d& pos,
                                  const Eigen::Vector3d& vel,
                                  const PositionSetpoint& sp, double dt);

 private:
  // Clamp body_z to within tilt_max of vertical (0,0,1), preserving heading.
  void limitTilt(Eigen::Vector3d& body_z) const;

  PositionControllerCfg cfg_;
  Pid<double, 3> vel_pid_;
  PidState<double, 3> vel_state_;
  Mask sat_hi_;
  Mask sat_lo_;
};

}  // namespace apl

#endif  // APL_POSITION_CONTROLLER_HPP_
