#!/bin/bash
# Source the ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

echo "Starting ROS 2 Nodes..."

# Run GPIO node in the background
python3 gpio_node.py &
PID1=$!

# Run Battery node in the background
python3 canbat_node.py &
PID2=$!

# Keep the script running and wait for processes
wait $PID1 $PID2
