#!/bin/bash
set -e

# check ubuntu version and select appropriate ROS installation
function install_ros_humble() {
    bash ./ros-humble-install.sh
}

function install_ros_jazzy() {
    bash ./ros-jazzy-install.sh
}

# determine if ros2 is already available
if command -v ros2 >/dev/null 2>&1; then
    echo "ROS 2 already installed"
else
    # detect ubuntu codename
    UBUNTU_CODENAME=$(source /etc/os-release && echo $UBUNTU_CODENAME)
    case "$UBUNTU_CODENAME" in
        jammy|jammy-*)
            echo "Installing ROS 2 Humble for Ubuntu $UBUNTU_CODENAME"
            install_ros_humble
            ;;
        lunar|lunar-*)
            echo "Installing ROS 2 Jazzy for Ubuntu $UBUNTU_CODENAME"
            install_ros_jazzy
            ;;
        *)
            echo "Ubuntu version $UBUNTU_CODENAME not recognized. Defaulting to Humble."
            install_ros_humble
            ;;
    esac
fi

# The path where your install script puts the compiled agent
AGENT_EXEC="$HOME/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"

if [ -f "$AGENT_EXEC" ]; then
    echo "Micro ROS already installed globally in ~/microros_ws!"
else
    echo "Micro ROS agent not found. Running installation script..."
    bash setup/uros-install.sh
fi

cd ../

rosdep update

rosdep install --from-paths src -y