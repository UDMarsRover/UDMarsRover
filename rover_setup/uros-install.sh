#!/bin/bash
set -e

# Define the ROS 2 distro (change if using foxy, galactic, iron, etc.)
ROS_DISTRO=${ROS_DISTRO:-humble}
WORKSPACE_DIR=~/microros_ws

echo "Starting micro-ROS agent installation for ROS 2 $ROS_DISTRO..."

# Create workspace directory if it doesn't exist
mkdir -p $WORKSPACE_DIR/src
cd $WORKSPACE_DIR

# Clone the micro-ROS setup repository
if [ ! -d "src/micro_ros_setup" ]; then
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

# Update dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the micro-ROS setup package
colcon build

# Source the workspace
source install/local_setup.bash

# Create the agent
ros2 run micro_ros_setup create_agent_ws.sh

# Build the agent
ros2 run micro_ros_setup build_agent.sh

# Add source command to .bashrc if not present
grep -qxF "source $WORKSPACE_DIR/install/local_setup.bash" ~/.bashrc || echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc

echo "micro-ROS agent installation complete."
echo "Please run 'source ~/.bashrc' or open a new terminal."