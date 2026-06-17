#ifndef APL20_ROS_MISSION_PLAYER_HPP_
#define APL20_ROS_MISSION_PLAYER_HPP_

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "apl/mission.hpp"

namespace apl20_ros {

// The "planner" half of the controller/commander split, fully decoupled from
// the autopilot. It builds a waypoint pattern (hover, square, lawnmower) in
// absolute local ENU and publishes it ONCE as a latched nav_msgs/Path on
// setpoint_path/local; the autopilot then sequences + tracks it (advancement
// and tracking live there now). No pose feedback, no per-cycle streaming -- so
// it runs in its own launch and can be killed once the path is delivered (the
// autopilot keeps following the last path it received).
//
// Built to grow into a planner: replace buildPath() (a static pattern) with a
// planner that republishes nav_msgs/Path, and nothing else changes.
class MissionPlayer : public rclcpp::Node {
 public:
  explicit MissionPlayer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  nav_msgs::msg::Path buildPath() const;

  std::string pattern_ = "hover";
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_altitude_ = 2.0;
  double target_yaw_ = 0.0;
  double square_side_ = 4.0;
  double mow_length_ = 10.0;
  double mow_width_ = 6.0;
  double mow_spacing_ = 3.0;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace apl20_ros

#endif  // APL20_ROS_MISSION_PLAYER_HPP_
