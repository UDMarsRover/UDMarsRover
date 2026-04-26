#!/bin/bash

cd ../

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/microros_ws/install/local_setup.bash
source install/local_setup.bash

ros2 launch rover rover_launch.py