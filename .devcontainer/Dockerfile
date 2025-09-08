# Start with the official ROS 2 Humble base image
FROM ros:humble-ros-base

# Use root user for setup
USER root

# Install necessary packages for building ROS 2 packages and sudo
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    ros-$ROS_DISTRO-demo-nodes-py \
    iputils-ping \
    iproute2 \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user with a specific UID/GID for consistency
ARG USERNAME=udmrt
ARG USER_UID=1000
ARG USER_GID=1000

# Create the group and user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add the user to the sudo group to grant sudo access
    && usermod -aG sudo $USERNAME \
    # Configure sudo to not require a password for this user
    && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set the working directory for the user
WORKDIR /home/$USERNAME

# Switch to the non-root user
USER $USERNAME

# Use bash as the shell
SHELL ["/bin/bash", "-c"]

# Source the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc