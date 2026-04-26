 #!/bin/bash

echo "--- 🛠️  ROVER SYSTEM INITIALIZATION ---"

# 1. FAILSAFE: Install joy package if missing
if ! ros2 pkg list | grep -q "^joy$"; then
    echo "⚠️  Joy package not found. Installing..."
    # 'export DEBIAN_FRONTEND=noninteractive' prevents the installer from asking questions
    export DEBIAN_FRONTEND=noninteractive
    apt update && apt install ros-humble-joy -y
    echo "✅ Joy package installed."
else
    echo "✅ Joy package already present."
fi


# 2. FIND AND SET JOYSTICK PERMISSIONS

# The F310 in 'X' mode usually shows up as /dev/input/js0
JOY_DEV=$(ls /dev/input/event0 2>/dev/null | head -n 1)

if [ -z "$JOY_DEV" ]; then
    echo "❌ ERROR: No joystick found! Check PowerShell 'usbipd attach' and ensure switch is on 'X'."
    # We don't exit here so the Agent can still start if you want to test motor code
else
    echo "✅ Found Joystick at $JOY_DEV. Setting permissions..."
    chmod a+rw $JOY_DEV
fi


# 3. START MICRO-ROS AGENT
echo "3. Starting micro-ROS Agent..."
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 &
AGENT_PID=$!
sleep 5




# 4. START JOY NODE
echo "4. Starting Joy Node..."
ros2 run joy joy_node --ros-args -p device_id:=0 &
JOY_PID=$!
sleep 2


# 5. START TRANSLATOR
echo "5. Starting Python Translator..."
python3 src/translator.py &
TRANS_PID=$!


echo "--- 🚀 SYSTEM LIVE. DRIVE ON! ---"
echo "Press Ctrl+C to stop all nodes."


# CLEANUP: This ensures everything dies when you hit Ctrl+C
trap "kill $AGENT_PID $JOY_PID $TRANS_PID; echo 'Stopping nodes...'; exit" INT
wait 