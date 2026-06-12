#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "apl20_ros/autopilot_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<apl20_ros::AutopilotNode>());
  rclcpp::shutdown();
  return 0;
}
