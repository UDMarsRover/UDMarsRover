cmake_minimum_required(VERSION 3.8)
project(arm)

find_package(ament_cmake REQUIRED)

#
# Install robot description
#
install(
  DIRECTORY
    urdf
    launch
    rviz
    meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()