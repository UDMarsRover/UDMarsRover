#!/bin/bash
# Source the ROS 2 Jazzy setup file
source /opt/ros/jazzy/setup.bash

# Execute the command passed into this entrypoint
exec "$@"
