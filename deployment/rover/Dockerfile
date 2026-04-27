# Use the ROS jazzy base image
FROM ros:jazzy-ros-base

    # Install necessary build tools and git
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Copy the ROS package into the container
WORKDIR /home/ros_ws/

COPY src/ src/

RUN colcon build

CMD ["bash"]    