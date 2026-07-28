# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Install Python, Bluetooth dependencies, and pip
# (Adjust python3 dependencies if you are using a specific library like 'bleak' or 'pybluez')
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-bluez \
    bluez \
    libbluetooth-dev \
    && rm -rf /var/lib/apt/lists/*

#Add bleak
RUN pip3 install --no-cache-dir bleak setuptools --break-system-packages


# Set the working directory
WORKDIR /app

# Copy your script and entrypoint
COPY bt_node.py .
COPY entrypoint.sh /

# Install any required Python packages your script needs
# RUN pip3 install --no-cache-dir bleak setuptools --break-system-packages

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command to run your script
CMD ["python3", "bt_node.py"]
