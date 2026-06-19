cmake_minimum_required(VERSION 3.8)
project(arm_ik_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)

#
# Main IK Node
#
add_executable(ik_node src/ik_node.cpp)

ament_target_dependencies(
  ik_node
  rclcpp
  sensor_msgs
)

target_link_libraries(ik_node
  orocos-kdl
)

#
# Joystick Test Node
#
add_executable(joy_test src/joy_test.cpp)

ament_target_dependencies(
  joy_test
  rclcpp
  sensor_msgs
)

#
# Trac Ik test
#
add_executable(tracik_test src/tracik_test.cpp)

ament_target_dependencies(
  tracik_test
  rclcpp
)

target_link_libraries(tracik_test
  orocos-kdl
  trac_ik_lib::trac_ik
)

#
# Install executables
#
install(TARGETS
  ik_node
  joy_test
  tracik_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()