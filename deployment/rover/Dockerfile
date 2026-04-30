# Use the ROS jazzy base image
FROM ros:jazzy-ros-base

# Install necessary packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-joy \
    ros-jazzy-robot-localization

# Copy the ROS package into the container
WORKDIR /home/ros_ws/

COPY src/ src/

RUN colcon build

CMD ["bash", "-c", "source install/setup.bash && ros2 launch rover rover_launch.py"]