#include "apl20_ros/autopilot_node.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace apl20_ros {

namespace {
// Stream setpoints for ~1 s (at the odometry rate) before requesting offboard
// and arming, satisfying PX4's COM_OF_LOSS_T freshness rule.
constexpr int kSetpointsBeforeArm = 200;
// Retry the mode/arm request every this many samples until it takes.
constexpr int kRetryInterval = 100;

// PX4 SITL x500 (airframe 4001_gz_x500): the airframe geometry, one rotor per
// motor in ActuatorMotors output order. Motors 0,1 spin CCW (km > 0), 2,3 CW.
constexpr std::array<apl::Rotor<double>, 4> kX500Geometry = {{
    {.arm_x = 0.13, .arm_y = 0.22, .km = 0.05},    // 0: front-left,  CCW
    {.arm_x = -0.13, .arm_y = -0.20, .km = 0.05},  // 1: rear-right,  CCW
    {.arm_x = 0.13, .arm_y = -0.22, .km = -0.05},  // 2: front-right, CW
    {.arm_x = -0.13, .arm_y = 0.20, .km = -0.05},  // 3: rear-left,   CW
}};

// Heading [rad] from a body->world quaternion (the yaw of its ZYX Euler
// angles). In ENU this is the heading about +z, CCW from +x (east).
double Heading(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
}  // namespace

AutopilotNode::AutopilotNode(const rclcpp::NodeOptions& options)
    : Node("autopilot", options),
      position_(loadPositionCfg()),
      attitude_ref_(loadAttitudeReferenceCfg()),
      attitude_(loadAttitudeCfg()),
      rate_(loadRateCfg()),
      allocator_(loadAllocatorCfg()),
      position_ref_(loadPositionReferenceCfg()),
      recorder_(declare_parameter<std::string>("track_log", std::string(""))) {
  seed_thrust_ = position_.cfg().hover_thrust;
  ff_threshold_ = attitude_ref_.cfg().ff_threshold;
  takeoff_alt_ = declare_parameter<double>("takeoff_alt", 0.5);
  setpoint_timeout_ = declare_parameter<double>("setpoint_timeout", 0.5);
  accept_radius_ = declare_parameter<double>("accept_radius", 0.4);
  settle_speed_ = declare_parameter<double>("settle_speed", 0.4);
  // Off by default: a real vehicle only ever arms on an explicit operator
  // command. Set true for headless SITL to self-command offboard + arm.
  auto_engage_ = declare_parameter<bool>("auto_engage", false);

  // Setpoints in: MAVROS-style topics, most-recent level wins (see the class
  // comment). Plain default QoS -- these come from a commander, not PX4.
  pos_sp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/setpoint_position/local", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped& msg) {
        onPositionSetpoint(msg);
      });
  att_sp_sub_ = create_subscription<msg::AttitudeTarget>(
      "~/setpoint_raw/attitude", rclcpp::SensorDataQoS(),
      [this](const msg::AttitudeTarget& msg) { onAttitudeSetpoint(msg); });
  // A Path is a multi-waypoint mission: latched (transient-local) so a
  // commander can publish it once and a later-starting autopilot still receives
  // it.
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "~/setpoint_path/local", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::Path& msg) { onPath(msg); });

  // Vehicle pose out, for a PX4-agnostic commander (MAVROS
  // local_position/pose).
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/local_position/pose", rclcpp::SensorDataQoS());

  // PX4's uXRCE-DDS client streams topics best-effort; match it on both sides.
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos);
  motors_pub_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
      "/fmu/in/actuator_motors", qos);
  // Thrust/torque setpoints mirror the actuator command purely so PX4's land
  // detector (and logging) see the true throttle -- see publishThrustTorque.
  thrust_sp_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", qos);
  torque_sp_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", qos);
  cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,  // v1.17 publishes the versioned topic
      [this](const px4_msgs::msg::VehicleStatus& msg) {
        arming_state_ = msg.arming_state;
        nav_state_ = msg.nav_state;
      });
  local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const px4_msgs::msg::VehicleLocalPosition& msg) {
        onLocalPosition(msg);
      });
  // Odometry drives the control loop: attitude + body rates and the clock.
  odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      [this](const px4_msgs::msg::VehicleOdometry& msg) { onOdometry(msg); });
}

Eigen::Vector3d AutopilotNode::declareVec3(const std::string& name,
                                           const Eigen::Vector3d& def) {
  const std::vector<double> v =
      declare_parameter<std::vector<double>>(name, {def.x(), def.y(), def.z()});
  if (v.size() != 3) {
    throw std::invalid_argument(name + " must have exactly 3 elements");
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

apl::PositionControllerCfg AutopilotNode::loadPositionCfg() {
  apl::PositionControllerCfg cfg;
  cfg.kp_pos = declareVec3("pos.kp", Eigen::Vector3d(0.95, 0.95, 1.0));
  // Velocity loop: the integral (vel_ki) absorbs the unknown hover thrust.
  cfg.vel.kp =
      declareVec3("pos.vel_kp", Eigen::Vector3d(1.8, 1.8, 4.0)).array();
  cfg.vel.ki =
      declareVec3("pos.vel_ki", Eigen::Vector3d(0.4, 0.4, 2.0)).array();
  cfg.vel.i_max =
      declareVec3("pos.vel_imax", Eigen::Vector3d(4.0, 4.0, 9.81)).array();
  cfg.hover_thrust = declare_parameter<double>("hover_thrust", 0.70);
  cfg.tilt_max = declare_parameter<double>("pos.tilt_max", 0.6);
  cfg.thrust_min = declare_parameter<double>("pos.thrust_min", 0.1);
  cfg.thrust_max = declare_parameter<double>("pos.thrust_max", 0.9);
  return cfg;
}

apl::PositionReferenceCfg AutopilotNode::loadPositionReferenceCfg() {
  apl::PositionReferenceCfg cfg;
  cfg.kp_pos = declare_parameter<double>("posref.kp_pos", 1.0);
  cfg.kp_vel = declare_parameter<double>("posref.kp_vel", 2.0);
  cfg.vel_max = declareVec3("posref.vel_max", Eigen::Vector3d(3.0, 3.0, 1.5));
  cfg.accel_max =
      declareVec3("posref.accel_max", Eigen::Vector3d(3.0, 3.0, 3.0));
  cfg.jerk_max = declareVec3("posref.jerk_max", Eigen::Vector3d(8.0, 8.0, 8.0));
  cfg.yaw_rate_max = declare_parameter<double>("posref.yaw_rate_max", 0.5);
  return cfg;
}

apl::AttitudeReferenceCfg AutopilotNode::loadAttitudeReferenceCfg() {
  apl::AttitudeReferenceCfg cfg;
  cfg.input_tc = declare_parameter<double>("attref.input_tc", 0.15);
  cfg.ang_accel_max =
      declareVec3("attref.ang_accel_max", Eigen::Vector3d(19.2, 19.2, 4.71));
  cfg.ang_vel_max =
      declareVec3("attref.ang_vel_max", Eigen::Vector3d(20.0, 20.0, 20.0));
  cfg.ff_threshold = declare_parameter<double>("attref.ff_threshold", 0.5236);
  cfg.feedforward = declare_parameter<bool>("attref.feedforward", true);
  return cfg;
}

apl::AttitudeControllerCfg AutopilotNode::loadAttitudeCfg() {
  apl::AttitudeControllerCfg cfg;
  cfg.kp = declareVec3("att.kp", Eigen::Vector3d(6.5, 6.5, 2.8));
  cfg.rate_limit =
      declareVec3("att.rate_limit", Eigen::Vector3d(3.84, 3.84, 3.49));
  return cfg;
}

apl::RateControllerCfg AutopilotNode::loadRateCfg() {
  apl::RateControllerCfg cfg;
  cfg.pid.kp =
      declareVec3("rate.kp", Eigen::Vector3d(0.15, 0.15, 0.20)).array();
  cfg.pid.ki =
      declareVec3("rate.ki", Eigen::Vector3d(0.20, 0.20, 0.10)).array();
  cfg.pid.kd =
      declareVec3("rate.kd", Eigen::Vector3d(0.003, 0.003, 0.0)).array();
  cfg.pid.i_max =
      declareVec3("rate.i_max", Eigen::Vector3d(0.3, 0.3, 0.3)).array();
  cfg.pid.d_lpf_hz =
      declareVec3("rate.d_lpf_hz", Eigen::Vector3d(40.0, 40.0, 40.0)).array();
  cfg.output_limit = declare_parameter<double>("rate.output_limit", 1.0);
  return cfg;
}

apl::ControlAllocatorCfg<double, 4> AutopilotNode::loadAllocatorCfg() {
  auto cfg = apl::MakeControlAllocatorCfg(kX500Geometry);
  cfg.motor_min = declare_parameter<double>("motor_idle", 0.0);
  return cfg;
}

void AutopilotNode::onPositionSetpoint(
    const geometry_msgs::msg::PoseStamped& msg) {
  const Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x,
                             msg.pose.orientation.y, msg.pose.orientation.z);
  // On entering position mode, re-seed the trajectory shaper at the current
  // pose so the reference starts where the vehicle is (no jump from a stale
  // target left over from another mode).
  if (mode_ != Mode::kPosition && captured_) {
    position_ref_.reset(pos_enu_, vel_enu_, heading_);
  }
  sp_pos_ = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y,
                            msg.pose.position.z);
  sp_yaw_ = Heading(q);
  mode_ = Mode::kPosition;
  setpoint_stamp_ = now();
}

void AutopilotNode::onPath(const nav_msgs::msg::Path& msg) {
  if (msg.poses.empty()) {
    return;
  }
  path_.clear();
  path_.reserve(msg.poses.size());
  for (const geometry_msgs::msg::PoseStamped& p : msg.poses) {
    const Eigen::Quaterniond q(p.pose.orientation.w, p.pose.orientation.x,
                               p.pose.orientation.y, p.pose.orientation.z);
    path_.push_back({Eigen::Vector3d(p.pose.position.x, p.pose.position.y,
                                     p.pose.position.z),
                     Heading(q)});
  }
  path_idx_ = 0;
  // On entering path mode, re-seed the shaper at the current pose (no jump from
  // a stale target left over from another mode).
  if (mode_ != Mode::kPath && captured_) {
    position_ref_.reset(pos_enu_, vel_enu_, heading_);
  }
  mode_ = Mode::kPath;
  setpoint_stamp_ = now();
  RCLCPP_INFO(get_logger(), "path: %zu waypoint(s)", path_.size());
}

void AutopilotNode::onAttitudeSetpoint(const msg::AttitudeTarget& m) {
  using Msg = msg::AttitudeTarget;
  if (!(m.type_mask & Msg::IGNORE_THRUST)) {
    sp_thrust_ = m.thrust;
  }
  if (m.type_mask & Msg::IGNORE_ATTITUDE) {
    sp_rate_ = Eigen::Vector3d(m.body_rate.x, m.body_rate.y, m.body_rate.z);
    if (mode_ != Mode::kRate && captured_) {
      rate_.reset(sp_rate_);
    }
    mode_ = Mode::kRate;
  } else {
    sp_att_ = Eigen::Quaterniond(m.orientation.w, m.orientation.x,
                                 m.orientation.y, m.orientation.z)
                  .normalized();
    mode_ = Mode::kAttitude;
  }
  setpoint_stamp_ = now();
}

void AutopilotNode::onLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition& msg) {
  if (msg.xy_valid && msg.z_valid) {
    // PX4 publishes NED; the cascade is ENU.
    pos_enu_ = apl::InterconvertNedEnu(Eigen::Vector3d(msg.x, msg.y, msg.z));
    vel_enu_ = apl::InterconvertNedEnu(Eigen::Vector3d(msg.vx, msg.vy, msg.vz));
    have_local_ = true;
  }
}

void AutopilotNode::onOdometry(const px4_msgs::msg::VehicleOdometry& msg) {
  // PX4 odometry is FRD body -> NED world; convert to FLU body -> ENU world so
  // the whole cascade runs in ENU/FLU. This is the only attitude/rate boundary.
  const Eigen::Quaterniond q = apl::InterconvertAeroRos(
      Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]));  // w,x,y,z
  const Eigen::Vector3d rate_meas = apl::InterconvertFluFrd(
      Eigen::Vector3d(msg.angular_velocity[0], msg.angular_velocity[1],
                      msg.angular_velocity[2]));
  double dt = 0.004;
  if (last_sample_us_) {
    dt = static_cast<double>(msg.timestamp_sample - *last_sample_us_) * 1e-6;
  }
  last_sample_us_ = msg.timestamp_sample;
  // The HIL/uXRCE odometry timestamp_sample periodically jumps BACKWARD (~-340
  // ms when sim-time resyncs against the agent clock), yielding a negative dt
  // that detonates the rate PID's derivative/integral into a one-sample motor
  // spike -- the diagonal "dart". The state itself is continuous across the
  // jump, so clamp dt to a sane range and keep controlling; a bad timestamp
  // must never reach the cascade.
  if (dt <= 0.0 || dt > 0.05) {
    dt = 0.0107;  // ~93 Hz nominal
  }
  heading_ = Heading(q);

  // On the first valid local position, prime the loops so we are ready the
  // moment the operator engages (they are re-seeded again on that edge).
  if (have_local_ && !captured_) {
    seedControllers(q, rate_meas, msg.timestamp_sample);
    captured_ = true;
    RCLCPP_INFO(get_logger(), "Captured start pose; ready for setpoints.");
  }

  // Heartbeat + pose out every cycle (>2 Hz) to keep offboard *available* and
  // feed a commander -- streamed whether or not we are engaged.
  publishOffboardControlMode();
  if (have_local_) {
    publishPose(q, now());
  }

  // Drive the motors only once the vehicle is actually armed AND in offboard --
  // arming is the operator's/GCS's decision, never the node's, so it never
  // spins props on its own. Until then, command them explicitly stopped. On the
  // disengaged->engaged edge, re-seed the loops at the current pose so control
  // starts from where the vehicle is (no stale hover; the takeoff guard then
  // climbs from the ground).
  const bool engaged = captured_ && isEngaged();
  if (engaged) {
    if (!engaged_) {
      seedControllers(q, rate_meas, msg.timestamp_sample);
      RCLCPP_INFO(get_logger(), "ENGAGED (armed + offboard) -> controlling.");
    }
    const std::pair<Eigen::Vector3d, double> out = step(q, rate_meas, dt);
    publishMotors(out.first, out.second);
  } else {
    publishMotorsStopped();
  }
  engaged_ = engaged;

  ++setpoint_count_;
  if (auto_engage_) {
    maybeRequestOffboardArm();
  }
}

std::pair<Eigen::Vector3d, double> AutopilotNode::step(
    const Eigen::Quaterniond& q, const Eigen::Vector3d& rate_meas, double dt) {
  // Resolve the effective mode: an attitude/rate command that has gone stale is
  // unsafe to keep honoring, so fall back to a level idle. A position command
  // is safe to hold indefinitely (the shaper just keeps the last hover), so it
  // never expires -- a single `topic pub` is a persistent hover.
  Mode mode = mode_;
  const bool stale = setpoint_stamp_ &&
                     (now() - *setpoint_stamp_).seconds() > setpoint_timeout_;
  if ((mode == Mode::kAttitude || mode == Mode::kRate) && stale) {
    mode = Mode::kIdle;
  }

  switch (mode) {
    case Mode::kRate: {
      const Eigen::Vector3d torque =
          rate_.update(sp_rate_, rate_meas, sp_thrust_, dt);
      return {torque, sp_thrust_};
    }
    case Mode::kAttitude: {
      const Eigen::Vector3d torque =
          attitudeToTorque(q, sp_att_, rate_meas, sp_thrust_, dt);
      return {torque, sp_thrust_};
    }
    case Mode::kPosition:
      return trackPosition(sp_pos_, sp_yaw_, -1, q, rate_meas, dt);
    case Mode::kPath: {
      // Sequence through the path once airborne: advance to the next waypoint
      // when settled within the acceptance radius (stop-and-go).
      if (airborne_ && path_idx_ + 1 < path_.size() &&
          (pos_enu_ - path_[path_idx_].pos).norm() < accept_radius_ &&
          vel_enu_.norm() < settle_speed_) {
        ++path_idx_;
        RCLCPP_INFO(get_logger(), "reached waypoint %zu/%zu", path_idx_,
                    path_.size() - 1);
      }
      return trackPosition(path_[path_idx_].pos, path_[path_idx_].yaw,
                           static_cast<int>(path_idx_), q, rate_meas, dt);
    }
    case Mode::kIdle:
    default:
      return {Eigen::Vector3d::Zero(), seed_thrust_};
  }
}

std::pair<Eigen::Vector3d, double> AutopilotNode::trackPosition(
    const Eigen::Vector3d& tgt_pos, double tgt_yaw, int wp,
    const Eigen::Quaterniond& q, const Eigen::Vector3d& rate_meas, double dt) {
  // Takeoff guard: until climbed past takeoff_alt, hold the start xy + heading
  // and only climb to the commanded altitude (a grounded vehicle cannot
  // yaw/translate without saturating the torque and starving the climb). Then
  // honor the target in full.
  Eigen::Vector3d pos = tgt_pos;
  double yaw = tgt_yaw;
  if (!airborne_) {
    pos = Eigen::Vector3d(start_pos_.x(), start_pos_.y(), tgt_pos.z());
    yaw = start_yaw_;
    wp = -1;  // the takeoff climb is not a waypoint
    if ((pos_enu_.z() - start_pos_.z()) > takeoff_alt_) {
      airborne_ = true;
      RCLCPP_INFO(get_logger(), "AIRBORNE -> tracking setpoints");
    }
  }
  const apl::PositionSetpoint sp = position_ref_.update(pos, yaw, dt);
  if (recorder_.enabled() && flight_start_us_ && last_sample_us_) {
    const double t =
        static_cast<double>(*last_sample_us_ - *flight_start_us_) * 1e-6;
    recorder_.record(t, wp, sp.position, sp.yaw, pos_enu_, heading_);
  }
  const apl::PositionControllerOutput pos_out =
      position_.update(pos_enu_, vel_enu_, sp, dt);
  const Eigen::Vector3d torque = attitudeToTorque(
      q, pos_out.attitude_setpoint, rate_meas, pos_out.collective_thrust, dt);
  return {torque, pos_out.collective_thrust};
}

Eigen::Vector3d AutopilotNode::attitudeToTorque(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& qd,
    const Eigen::Vector3d& rate_meas, double collective, double dt) {
  // Shape the desired attitude into an achievable target + feedforward, run the
  // attitude feedback against the target, then combine (the feedforward is
  // de-prioritized as the tilt error grows) and close the rate loop.
  const apl::AttitudeSetpoint att_sp = attitude_ref_.update(qd, 0.0, dt);
  const Eigen::Vector3d corrective = attitude_.update(q, att_sp.attitude);
  const double thrust_err = apl::ThrustErrorAngle(q, att_sp.attitude);
  const Eigen::Vector3d rate_sp = apl::CombineAttitudeRate(
      corrective, att_sp.ang_vel_ff, rate_meas, thrust_err, ff_threshold_);
  return rate_.update(rate_sp, rate_meas, collective, dt);
}

void AutopilotNode::publishOffboardControlMode() {
  px4_msgs::msg::OffboardControlMode m;
  m.timestamp = nowUs();
  m.direct_actuator = true;  // we allocate to motors; PX4 runs no control
  ocm_pub_->publish(m);
}

void AutopilotNode::publishMotors(const Eigen::Vector3d& torque,
                                  double thrust) {
  // The cascade emits FLU body torque; the allocator mirrors PX4's FRD rotor
  // geometry, so convert at this (actuator) boundary. Collective is unsigned.
  const Eigen::Vector3d torque_frd = apl::InterconvertFluFrd(torque);
  const Eigen::Vector4d motors = allocator_.allocate(torque_frd, thrust);
  const uint64_t t = nowUs();
  px4_msgs::msg::ActuatorMotors m;
  m.timestamp = t;
  m.timestamp_sample = t;
  m.control.fill(std::numeric_limits<float>::quiet_NaN());  // unused -> stopped
  for (int i = 0; i < motors.size(); ++i) {
    m.control[i] = static_cast<float>(motors[i]);
  }
  motors_pub_->publish(m);
  publishThrustTorque(torque_frd, thrust, t);
}

void AutopilotNode::publishThrustTorque(const Eigen::Vector3d& torque_frd,
                                        double thrust, uint64_t stamp_us) {
  // PX4 normalizes multicopter thrust along body -z (up), so z = -collective;
  // the land detector reads throttle = -xyz[2]. Only ever published while
  // engaged (from publishMotors) -- when not engaged PX4 owns these topics.
  px4_msgs::msg::VehicleThrustSetpoint ts;
  ts.timestamp = stamp_us;
  ts.timestamp_sample = stamp_us;
  ts.xyz = {0.0F, 0.0F, static_cast<float>(-thrust)};
  thrust_sp_pub_->publish(ts);

  px4_msgs::msg::VehicleTorqueSetpoint qs;
  qs.timestamp = stamp_us;
  qs.timestamp_sample = stamp_us;
  Eigen::Vector3f::Map(qs.xyz.data()) = torque_frd.cast<float>();
  torque_sp_pub_->publish(qs);
}

void AutopilotNode::publishMotorsStopped() {
  // All-NaN control is ActuatorMotors' "stopped" sentinel: the props stay off
  // while the offboard heartbeat keeps offboard selectable for the operator.
  const uint64_t t = nowUs();
  px4_msgs::msg::ActuatorMotors m;
  m.timestamp = t;
  m.timestamp_sample = t;
  m.control.fill(std::numeric_limits<float>::quiet_NaN());
  motors_pub_->publish(m);
}

void AutopilotNode::seedControllers(const Eigen::Quaterniond& q,
                                    const Eigen::Vector3d& rate_meas,
                                    uint64_t sample_us) {
  start_pos_ = pos_enu_;
  start_yaw_ = Heading(q);
  airborne_ = false;
  position_ref_.reset(pos_enu_, vel_enu_, start_yaw_);
  position_.reset(vel_enu_);
  attitude_ref_.reset(q);
  rate_.reset(rate_meas);
  flight_start_us_ = sample_us;
}

bool AutopilotNode::isEngaged() const {
  return arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
         nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

void AutopilotNode::publishPose(const Eigen::Quaterniond& q,
                                const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = stamp;
  p.header.frame_id = "map";  // local ENU (x east, y north, z up)
  p.pose.position.x = pos_enu_.x();
  p.pose.position.y = pos_enu_.y();
  p.pose.position.z = pos_enu_.z();
  p.pose.orientation.w = q.w();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  pose_pub_->publish(p);
}

void AutopilotNode::sendVehicleCommand(uint16_t command, float param1,
                                       float param2) {
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp = nowUs();
  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  cmd_pub_->publish(cmd);
}

void AutopilotNode::maybeRequestOffboardArm() {
  const bool offboard =
      nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  const bool armed =
      arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  if (setpoint_count_ >= kSetpointsBeforeArm && !(offboard && armed) &&
      setpoint_count_ % kRetryInterval == 0) {
    if (!offboard) {
      sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                         1.0F, 6.0F);  // base_mode=custom, main_mode=OFFBOARD
    }
    if (!armed) {
      sendVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F,
          0.0F);
    }
  }
}

uint64_t AutopilotNode::nowUs() const {
  return static_cast<uint64_t>(now().nanoseconds() / 1000);
}

}  // namespace apl20_ros
