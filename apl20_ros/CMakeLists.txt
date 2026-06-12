cmake_minimum_required(VERSION 3.8)
project(apl20_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(apl20 REQUIRED) # pure-math control kernels (non-ROS package)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# --- messages ----------------------------------------------------------------
# AttitudeTarget mirrors mavros_msgs/AttitudeTarget so a commander can drive the
# attitude / body-rate level of the cascade with the familiar fields.
rosidl_generate_interfaces(${PROJECT_NAME} "msg/AttitudeTarget.msg" DEPENDENCIES
                           std_msgs geometry_msgs)
# Handle for linking the generated C++ types into this package's own nodes.
rosidl_get_typesupport_target(cpp_typesupport "${PROJECT_NAME}"
                              rosidl_typesupport_cpp)

# --- autopilot_node: the offboard cascade controller -------------------------
add_executable(autopilot_node src/main_autopilot.cpp src/autopilot_node.cpp
                              src/tracking_recorder.cpp)
target_compile_features(autopilot_node PUBLIC cxx_std_20) # apl20 headers are C++20
target_include_directories(
  autopilot_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  autopilot_node apl20::rate_control apl20::attitude_control
  apl20::control_allocation apl20::position_control "${cpp_typesupport}")
ament_target_dependencies(autopilot_node "rclcpp" "std_msgs" "geometry_msgs"
                          "nav_msgs" "px4_msgs")

# --- mission_player: the commander (talks only the autopilot's topics) --------
add_executable(mission_player src/main_mission_player.cpp src/mission_player.cpp)
target_compile_features(mission_player PUBLIC cxx_std_20)
target_include_directories(
  mission_player PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(mission_player apl20::mission)
ament_target_dependencies(mission_player "rclcpp" "geometry_msgs" "nav_msgs")

install(TARGETS autopilot_node mission_player DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/track_analyze.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Linting is handled project-wide by clang-format / clang-tidy via pre-commit
# (see .pre-commit-config.yaml, .clang-tidy), not ament_lint. Node unit tests,
# when added, go under an if(BUILD_TESTING) block here.

ament_package()
