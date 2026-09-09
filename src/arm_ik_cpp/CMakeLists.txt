cmake_minimum_required(VERSION 3.8)
project(arm_ik_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#################################################
# Dependencies
#################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)

find_package(std_msgs REQUIRED)

#################################################
# IK Node
#################################################

add_executable(ik_node
  src/ik_node.cpp
)

ament_target_dependencies(
  ik_node
  rclcpp
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(
  ik_node
  orocos-kdl
  trac_ik_lib::trac_ik
)

#################################################
# Joystick Test
#################################################

add_executable(joy_test
  src/joy_test.cpp
)

ament_target_dependencies(
  joy_test
  rclcpp
  sensor_msgs
)

#################################################
# TRAC-IK Test
#################################################

add_executable(tracik_test
  src/tracik_test.cpp
)

ament_target_dependencies(
  tracik_test
  rclcpp
  ament_index_cpp
)

target_link_libraries(
  tracik_test
  orocos-kdl
  trac_ik_lib::trac_ik
)

#################################################
# arm_control
#################################################

add_executable(arm_control src/arm_control.cpp)

ament_target_dependencies(
    arm_control
    rclcpp
    sensor_msgs
    std_msgs
)

#################################################
# Install
#################################################

install(TARGETS
  ik_node
  joy_test
  tracik_test
  arm_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()