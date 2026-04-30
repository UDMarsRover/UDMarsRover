# Use the official ROS 2 Humble base image for ARM64
FROM ros:humble-ros-base

# 1. Install system dependencies (Minimizing for a non-GStreamer build)
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    unzip \
    rsync \
    cmake \
    build-essential \
    ros-dev-tools \
    # Audio Support (as requested)
    libasound2-dev \
    portaudio19-dev \
    libffi-dev \
    libssl-dev \
    #Protobuf
    protobuf-compiler \
    libprotobuf-dev \
    # ROS 2 Humble essentials
    ros-humble-common-interfaces \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# 2. Setup Workspace
WORKDIR /ros2_ws/src

# 3. Clone repository
RUN git clone -b ros2 --recurse-submodules https://github.com/introlab/opentera-webrtc-ros.git && \
    git clone https://github.com/introlab/audio_utils.git

# 4. Install Python WebRTC stack (aiortc)
# We install these directly to ensure the Python client has what it needs
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
    aiortc \
    aiohttp \
    PyAudio \
    opencv-python \
    "numpy<2" \
    # Install specific requirements from the client package
    -r opentera-webrtc-ros/opentera_client_ros/requirements.txt

# 5. THE BUILD: No GStreamer, but including Audio
WORKDIR /ros2_ws

RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    # The '|| true' ensures the build continues even if rosdep can't find 'rtabmap' etc.
    rosdep install --from-paths src --ignore-src -y --rosdistro humble || true && \
    # We only build the core and the python client, skipping all the problematic C++ nodes
    MAKEFLAGS="-j1 -l1" colcon build \
    --executor sequential \
    --parallel-workers 1 \
    --packages-up-to opentera_webrtc_ros_msgs opentera_client_ros \
    --packages-select audio_utils_msgs opentera_protobuf_messages opentera_webrtc_ros_msgs \   
    --packages-skip opentera_webrtc_ros_gstreamer map_image_generator \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

# Source the workspace for every new shell
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc


# 6. Copy your Python script into the container's workspace
COPY webrtc_ingest.py /ros2_ws/webrtc_ingest.py

# 7. Copy the entrypoint script into the container root
COPY entrypoint.sh /entrypoint.sh

# 8. Give the entrypoint script executable permissions
RUN chmod +x /entrypoint.sh



# 9. Set the new script as the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# 10. Set the default command to run your Python script
CMD ["python3", "/ros2_ws/webrtc_ingest.py"]

#ENTRYPOINT ["/ros2_ws/install/setup.bash"]
#CMD ["bash"]
