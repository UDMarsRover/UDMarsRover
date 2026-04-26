# Use the ROS jazzy base image
FROM ros:jazzy-ros-base

# Install necessary packages
RUN apt-get update

# Copy the ROS package into the container
WORKDIR /home/ros_ws/

COPY src/ src/

CMD ["bash"]    