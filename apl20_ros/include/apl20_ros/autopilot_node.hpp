#ifndef APL20_ROS_AUTOPILOT_NODE_HPP_
#define APL20_ROS_AUTOPILOT_NODE_HPP_

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "apl/attitude_controller.hpp"
#include "apl/attitude_reference.hpp"
#include "apl/control_allocator.hpp"
#include "apl/conversions.hpp"
#include "apl/position_controller.hpp"
#include "apl/position_reference.hpp"
#include "apl/rate_controller.hpp"
#include "apl20_ros/msg/attitude_target.hpp"
#include "apl20_ros/tracking_recorder.hpp"

namespace apl20_ros {

// The apl20 autopilot: one offboard cascade controller that PX4 runs at the
// direct-actuator (per-motor) level over uXRCE-DDS -- our ControlAllocator does
// the mixing PX4 would otherwise do, so PX4 runs none of its own control.
//
// It tracks setpoints; it does not plan. A commander (MissionPlayer, a teleop,
// or a bare `ros2 topic pub`) supplies them on MAVROS-style topics, and the
// most recently received one selects the active mode:
//
//   ~/setpoint_position/local (geometry_msgs/PoseStamped, ENU; yaw from the
//       quaternion heading)  -> PositionReference -> the full cascade.
//   ~/setpoint_path/local    (nav_msgs/Path, latched) -> a multi-waypoint
//       mission the autopilot sequences through (advancing once settled within
//       accept_radius), each leg shaped by PositionReference.
//   ~/setpoint_raw/attitude  (apl20_ros/AttitudeTarget) -> the attitude
//       controller (orientation + thrust) or, with IGNORE_ATTITUDE, the rate
//       controller directly (body_rate + thrust).
//
// VehicleLocalPosition (PX4 NED) and VehicleOdometry (PX4 FRD->NED attitude q +
// FRD body rates) are converted to ENU/FLU at ingestion (the only frame
// boundary; the whole cascade is ENU/FLU). VehicleLocalPosition feeds the
// position loop; VehicleOdometry feeds the inner loops and is the loop clock.
// The motors stay commanded-stopped until the vehicle is actually armed AND in
// offboard -- arming is the operator's decision (e.g. via the GCS), never the
// node's. Once engaged, a takeoff guard holds the start xy + heading and only
// climbs past `takeoff_alt` (the grounded vehicle cannot yaw/translate without
// the saturated torque starving the climb), so a position setpoint lifts off
// cleanly. `auto_engage:=true` restores the self-command offboard+arm handshake
// for headless SITL.
// It republishes the vehicle pose on ~/local_position/pose and, if `track_log`
// is set, logs the shaped reference vs the measured pose to CSV.
class AutopilotNode : public rclcpp::Node {
 public:
  explicit AutopilotNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // Active setpoint level -- selected by the most recently received setpoint.
  enum class Mode { kIdle, kPosition, kPath, kAttitude, kRate };
  struct Waypoint {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();  // ENU [m]
    double yaw = 0.0;                               // heading [rad]
  };

  apl::PositionControllerCfg loadPositionCfg();
  apl::PositionReferenceCfg loadPositionReferenceCfg();
  apl::AttitudeReferenceCfg loadAttitudeReferenceCfg();
  apl::AttitudeControllerCfg loadAttitudeCfg();
  apl::RateControllerCfg loadRateCfg();
  apl::ControlAllocatorCfg<double, 4> loadAllocatorCfg();
  Eigen::Vector3d declareVec3(const std::string& name,
                              const Eigen::Vector3d& def);

  void onPositionSetpoint(const geometry_msgs::msg::PoseStamped& msg);
  void onPath(const nav_msgs::msg::Path& msg);
  void onAttitudeSetpoint(const msg::AttitudeTarget& msg);
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg);
  void onOdometry(const px4_msgs::msg::VehicleOdometry& msg);

  // Run the active mode for one sample and return (torque, collective thrust).
  std::pair<Eigen::Vector3d, double> step(const Eigen::Quaterniond& q,
                                          const Eigen::Vector3d& rate_meas,
                                          double dt);
  // Position-domain tracking shared by the position and path modes: takeoff
  // guard -> trajectory shaper -> position controller -> inner cascade, logging
  // the shaped reference vs the measured pose under waypoint index `wp` (-1 is
  // used while taking off so the climb does not pollute a waypoint's stats).
  std::pair<Eigen::Vector3d, double> trackPosition(
      const Eigen::Vector3d& tgt_pos, double tgt_yaw, int wp,
      const Eigen::Quaterniond& q, const Eigen::Vector3d& rate_meas, double dt);
  // Inner half of the cascade: a desired attitude `qd` + collective -> torque
  // (attitude reference shaping -> attitude feedback -> rate loop). Shared by
  // the position/path and attitude modes.
  Eigen::Vector3d attitudeToTorque(const Eigen::Quaterniond& q,
                                   const Eigen::Quaterniond& qd,
                                   const Eigen::Vector3d& rate_meas,
                                   double collective, double dt);

  void publishOffboardControlMode();
  void publishMotors(const Eigen::Vector3d& torque, double thrust);
  // Mirror the FRD collective/torque we are allocating onto the thrust/torque
  // setpoint topics. PX4 runs no allocation in direct-actuator offboard, so it
  // never populates these -- and the land detector reads the zero thrust
  // setpoint as a zero throttle and false-triggers "landed" on a settled hover
  // (auto-disarm). Publishing the true collective keeps its throttle honest.
  void publishThrustTorque(const Eigen::Vector3d& torque_frd, double thrust,
                           uint64_t stamp_us);
  // Command the motors explicitly stopped (all-NaN) -- streamed whenever we are
  // not engaged, so props stay off while the offboard heartbeat keeps flowing.
  void publishMotorsStopped();
  void publishPose(const Eigen::Quaterniond& q, const rclcpp::Time& stamp);
  void sendVehicleCommand(uint16_t command, float param1, float param2);
  void maybeRequestOffboardArm();
  // True only when PX4 reports the vehicle actually armed AND in offboard --
  // the gate for driving the motors.
  bool isEngaged() const;
  // Seed the shapers + loops at the current pose/rate: on the first fix, and
  // again on the disengaged->engaged edge so control starts from where we are.
  void seedControllers(const Eigen::Quaterniond& q,
                       const Eigen::Vector3d& rate_meas, uint64_t sample_us);
  uint64_t nowUs() const;

  apl::PositionController position_;
  apl::AttitudeReference attitude_ref_;
  apl::AttitudeController attitude_;
  apl::RateController rate_;
  apl::ControlAllocator<double, 4> allocator_;
  apl::PositionReference position_ref_;
  TrackingRecorder recorder_;
  double ff_threshold_ = 0.5235987755982988;  // tilt-error FF gate [rad]
  double seed_thrust_ = 0.70;  // collective held while idle / pre-prime

  // Active setpoint (whichever level last arrived), and when it arrived.
  Mode mode_ = Mode::kIdle;
  std::optional<rclcpp::Time> setpoint_stamp_;
  double setpoint_timeout_ = 0.5;  // [s] beyond which a setpoint is stale
  Eigen::Vector3d sp_pos_ = Eigen::Vector3d::Zero();  // ENU [m]
  double sp_yaw_ = 0.0;                               // [rad]
  Eigen::Quaterniond sp_att_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d sp_rate_ = Eigen::Vector3d::Zero();  // body [rad/s]
  double sp_thrust_ = 0.0;                             // collective [0,1]

  // Path (multi-waypoint) setpoint + stop-and-go sequencing.
  std::vector<Waypoint> path_;
  std::size_t path_idx_ = 0;
  double accept_radius_ = 0.4;  // advance within this of a waypoint [m]
  double settle_speed_ = 0.4;   // ... and slower than this [m/s]

  Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_enu_ = Eigen::Vector3d::Zero();
  double heading_ = 0.0;  // current vehicle heading [rad]
  bool have_local_ = false;

  // Takeoff guard: climb at the captured start xy + heading until this high
  // above the start, then honor position/path setpoints in full.
  Eigen::Vector3d start_pos_ = Eigen::Vector3d::Zero();
  double start_yaw_ = 0.0;
  double takeoff_alt_ = 0.5;
  bool airborne_ = false;
  bool captured_ = false;
  std::optional<uint64_t> flight_start_us_;  // t = 0 for the tracking log

  std::optional<uint64_t> last_sample_us_;
  int setpoint_count_ = 0;
  uint8_t arming_state_ = 0;
  uint8_t nav_state_ = 0;
  bool auto_engage_ = false;  // opt-in: self-command offboard + arm (SITL only)
  bool engaged_ = false;      // armed+offboard last cycle (engage-edge detect)

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sp_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<msg::AttitudeTarget>::SharedPtr att_sp_sub_;
  rclcpp::SubscriptionBase::SharedPtr odom_sub_;
  rclcpp::SubscriptionBase::SharedPtr local_pos_sub_;
  rclcpp::SubscriptionBase::SharedPtr status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr
      thrust_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      torque_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

}  // namespace apl20_ros

#endif  // APL20_ROS_AUTOPILOT_NODE_HPP_
