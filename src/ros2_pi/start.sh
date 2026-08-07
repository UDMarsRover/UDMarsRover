#!/bin/bash
source /opt/ros/jazzy/setup.bash

echo "Starting ROS 2 Nodes..."

python3 gpio_node.py &
PID1=$!

python3 canbat_node.py &
PID2=$!

# Add the new system monitor node
python3 sys_monitor_node.py &
PID3=$!

wait $PID1 $PID2 $PID3
