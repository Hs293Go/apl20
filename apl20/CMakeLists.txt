cmake_minimum_required(VERSION 3.28.0)
project(
  apl20
  LANGUAGES CXX
  VERSION 0.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

# Pure-math rate-control library: the Pid kernel (header-only) plus the
# RateController driver (one TU). No ROS, no message types — reusable anywhere
# Eigen is available. apl20_ros consumes this via find_package(apl20).
add_library(apl20_rate_control src/rate_controller.cpp)
add_library(apl20::rate_control ALIAS apl20_rate_control)
# Export the installed target as apl20::rate_control (matching the build alias),
# not apl20::apl20_rate_control. PIC so this static lib can be linked into the
# shared apl20_ros library.
set_target_properties(
  apl20_rate_control PROPERTIES EXPORT_NAME rate_control
                                POSITION_INDEPENDENT_CODE ON)
target_sources(
  apl20_rate_control
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/apl/pid.hpp
         include/apl/rate_controller.hpp)
target_compile_features(apl20_rate_control PUBLIC cxx_std_20)
target_link_libraries(apl20_rate_control PUBLIC Eigen3::Eigen)

# Shared math kernels: SO(3) log/exp, the SqrtController/SlewRate finite-time
# laws, AngularInputShaper. Header-only, Eigen-only -- used by both the attitude
# and position reference shapers, so it is its own target.
add_library(apl20_math INTERFACE)
add_library(apl20::math ALIAS apl20_math)
set_target_properties(apl20_math PROPERTIES EXPORT_NAME math)
target_sources(
  apl20_math
  INTERFACE FILE_SET
            HEADERS
            BASE_DIRS
            include
            FILES
            include/apl/math.hpp
            include/apl/filters.hpp)
target_compile_features(apl20_math INTERFACE cxx_std_20)
target_link_libraries(apl20_math INTERFACE Eigen3::Eigen)

# Brescianini quaternion attitude controller: header-only. The outer loop of the
# cascade (attitude -> rate setpoint).
add_library(apl20_attitude_control)
add_library(apl20::attitude_control ALIAS apl20_attitude_control)
set_target_properties(apl20_attitude_control PROPERTIES EXPORT_NAME
                                                        attitude_control)
target_sources(
  apl20_attitude_control
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/apl/attitude_controller.hpp
         include/apl/attitude_reference.hpp
  PRIVATE src/attitude_controller.cpp src/attitude_reference.cpp)
target_compile_features(apl20_attitude_control PUBLIC cxx_std_20)
target_link_libraries(apl20_attitude_control PUBLIC Eigen3::Eigen apl20_math)

# Geometric position controller: header-only. Reuses the Pid kernel (from
# rate_control) for the velocity loop, whose integral term gives robustness to
# mass/hover-thrust uncertainty. The outer loop (position -> attitude + thrust).
add_library(apl20_position_control)
add_library(apl20::position_control ALIAS apl20_position_control)
set_target_properties(apl20_position_control PROPERTIES EXPORT_NAME
                                                        position_control)
target_sources(
  apl20_position_control
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/apl/position_controller.hpp
         include/apl/position_reference.hpp
  PRIVATE src/position_controller.cpp src/position_reference.cpp)
target_compile_features(apl20_position_control PUBLIC cxx_std_20)
target_link_libraries(apl20_position_control PUBLIC apl20_rate_control
                                                    apl20_math)

# Control allocation: header-only. Maps the body torque + collective thrust to
# per-motor commands for direct-actuator output (PX4-compatible normalized mix).
add_library(apl20_control_allocation INTERFACE)
add_library(apl20::control_allocation ALIAS apl20_control_allocation)
set_target_properties(apl20_control_allocation PROPERTIES EXPORT_NAME
                                                          control_allocation)
target_sources(
  apl20_control_allocation INTERFACE FILE_SET HEADERS BASE_DIRS include FILES
                                     include/apl/control_allocator.hpp)
target_compile_features(apl20_control_allocation INTERFACE cxx_std_20)
target_link_libraries(apl20_control_allocation INTERFACE Eigen3::Eigen)
#
# Mission patterns: header-only. Waypoint + survey-pattern generators (square,
# lawnmower) that emit absolute-NED waypoint lists for waypoint navigation.
add_library(apl20_mission INTERFACE)
add_library(apl::mission ALIAS apl20_mission)
set_target_properties(apl20_mission PROPERTIES EXPORT_NAME mission)
target_sources(apl20_mission INTERFACE FILE_SET HEADERS BASE_DIRS include FILES
                                       include/apl/mission.hpp)
target_compile_features(apl20_mission INTERFACE cxx_std_20)
target_link_libraries(apl20_mission INTERFACE Eigen3::Eigen)

# --- install + export so downstreams can find_package(apl20) -----------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
  TARGETS apl20_rate_control apl20_math apl20_attitude_control
          apl20_position_control apl20_control_allocation apl20_mission
  EXPORT apl20Targets
  FILE_SET HEADERS
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT apl20Targets
  NAMESPACE apl20::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/apl20)

configure_package_config_file(
  cmake/apl20Config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/apl20Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/apl20)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/apl20ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/apl20Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/apl20ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/apl20)

# --- tests -------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  # Register a GoogleTest executable: build it, link the unit(s) under test plus
  # gtest_main (which provides main()), then let gtest_discover_tests register
  # each TEST() case as its own CTest test so `colcon test` reports per-case
  # results.
  function(apl_add_gtest name)
    add_executable(${name} test/${name}.cpp)
    target_link_libraries(${name} PRIVATE ${ARGN} GTest::gtest_main)
    gtest_discover_tests(${name})
  endfunction()

  apl_add_gtest(test_pid apl20::rate_control)
  apl_add_gtest(test_filters apl20::math)
  apl_add_gtest(test_attitude apl20::attitude_control)
  apl_add_gtest(test_attitude_reference apl20::attitude_control)
  apl_add_gtest(test_position apl20::position_control)
  apl_add_gtest(test_control_allocator apl20::control_allocation)
  apl_add_gtest(test_position_reference apl20::position_control)
  apl_add_gtest(test_mission apl::mission)
endif()
