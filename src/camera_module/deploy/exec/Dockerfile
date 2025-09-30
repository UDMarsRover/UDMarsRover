FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && \
    apt-get install -y python3-pip python3-libgpiod && \
    rm -rf /var/lib/apt/lists/*

# Copy camera_module package
WORKDIR /workspace
COPY src/camera_module /workspace/camera_module

# Install Python dependencies
RUN pip3 install -r /workspace/camera_module/requirements.txt || true


ENV GPIOZERO_PIN_FACTORY=pigpio
RUN colcon build

# Source ROS 2 setup and run servo node
# CMD ["sleep", "infinity"]
ENTRYPOINT [ "bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec ros2 run camera_module servo_node" ]
