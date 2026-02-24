#!/bin/bash
# ros-humble-install.sh
# Installs ROS 2 Humble Hawksbill on Ubuntu 22.04 (Jammy Jellyfish)

set -e

echo "Starting ROS 2 Humble Installation..."

# 1. Set locale to UTF-8
echo "Setting locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Setup Sources
echo "Setting up sources..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Update apt and install curl and gnupg2
sudo apt update && sudo apt install -y curl gnupg2 lsb-release

# Add the ROS 2 GPG key with apt-key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 packages
echo "Installing ROS 2 packages..."
sudo apt update
sudo apt upgrade -y

# Desktop Install (Recommended): ROS, RViz, demos, tutorials.
sudo apt install -y ros-humble-desktop

# Install development tools and rosdep
echo "Installing development tools and initializing rosdep..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update

# 4. Environment setup
echo "Sourcing ROS 2 setup script..."
# Source for current shell
source /opt/ros/humble/setup.bash

# Add source command to bashrc if not already present
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "Adding ROS 2 source command to ~/.bashrc"
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 Humble installation complete!"