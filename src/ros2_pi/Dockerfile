FROM ros:jazzy-ros-base

# Prevent interactive prompts during apt installations
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for Bluetooth and pip
RUN apt-get update && apt-get install -y \
    python3-pip \
    bluez \
    bluetooth \
    dbus \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries
# Bypasses Ubuntu 24.04's strict PEP 668 pip restrictions inside the container
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip3 install bleak gpiozero rpi-lgpio

# Set up the working directory
WORKDIR /app

# Copy the scripts into the container
COPY canbat_node.py .
COPY gpio_node.py .
COPY start.sh .

# Make the startup script executable
RUN chmod +x start.sh

# Run the startup script when the container launches
CMD ["/app/start.sh"]
