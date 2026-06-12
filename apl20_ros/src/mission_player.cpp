#include "apl20_ros/mission_player.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace apl20_ros {

namespace {
// A waypoint -> a PoseStamped (NED position + heading encoded as the yaw
// quaternion about the NED down axis).
geometry_msgs::msg::PoseStamped ToPose(const apl::Waypoint<double>& wp) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";  // local NED (x north, y east, z down)
  p.pose.position.x = wp.position.x();
  p.pose.position.y = wp.position.y();
  p.pose.position.z = wp.position.z();
  const Eigen::Quaterniond q(
      Eigen::AngleAxisd(wp.yaw, Eigen::Vector3d::UnitZ()));
  p.pose.orientation.w = q.w();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  return p;
}
}  // namespace

MissionPlayer::MissionPlayer(const rclcpp::NodeOptions& options)
    : Node("mission_player", options) {
  pattern_ = declare_parameter<std::string>("pattern", std::string("hover"));
  target_x_ = declare_parameter<double>("target.x", 0.0);
  target_y_ = declare_parameter<double>("target.y", 0.0);
  target_altitude_ = declare_parameter<double>("target.altitude", 2.0);
  target_yaw_ = declare_parameter<double>("target.yaw", 0.0);
  square_side_ = declare_parameter<double>("square.side", 4.0);
  mow_length_ = declare_parameter<double>("mow.length", 10.0);
  mow_width_ = declare_parameter<double>("mow.width", 6.0);
  mow_spacing_ = declare_parameter<double>("mow.spacing", 3.0);

  // Latched (transient-local): publish the path once and keep it, so the
  // autopilot still receives it whether it started before or after us.
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "setpoint_path/local", rclcpp::QoS(1).transient_local());
  const nav_msgs::msg::Path path = buildPath();
  path_pub_->publish(path);
  RCLCPP_INFO(get_logger(), "published '%s' path: %zu waypoint(s) (latched)",
              pattern_.c_str(), path.poses.size());
}

nav_msgs::msg::Path MissionPlayer::buildPath() const {
  // Pattern centred on the target pose, at the survey altitude (NED z = down).
  const Eigen::Vector3d at(target_x_, target_y_, -target_altitude_);
  std::vector<apl::Waypoint<double>> wps;
  if (pattern_ == "square") {
    wps = apl::SquarePattern(at, square_side_, target_yaw_);
  } else if (pattern_ == "lawnmower") {
    wps = apl::LawnmowerPattern(at, mow_length_, mow_width_, mow_spacing_,
                                target_yaw_);
  } else {  // hover: a single waypoint overhead the target
    wps = {apl::Waypoint<double>{at, target_yaw_}};
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = now();
  for (const apl::Waypoint<double>& wp : wps) {
    path.poses.push_back(ToPose(wp));
  }
  return path;
}

}  // namespace apl20_ros
