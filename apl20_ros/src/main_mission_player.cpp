#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "apl20_ros/mission_player.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<apl20_ros::MissionPlayer>());
  rclcpp::shutdown();
  return 0;
}
