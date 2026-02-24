# Mini Rover Setup

This directory contains helper scripts to install ROS 2, build the rover package, and launch the rover software. The goal is to make it easy for new users to get the rover up and running on a fresh machine.

## Prerequisites

- Ubuntu 22.04 (Jammy) or 24.04 (Lunar) depending on ROS 2 distribution.
- A USB connection to the rover's microcontroller (ttyACM0 by default).
- A game controller (joystick) connected to the computer for teleoperation.

## Installation

1. Open a terminal and navigate to the project root:
	```bash
	cd ~/UDMarsRover/rover_setup
	```

2. Run the main installer script:
	```bash
	./install.sh
	```

	The script will first check whether `ros2` is already installed. If not it inspects the Ubuntu codename (`jammy`, `lunar`, etc.) and runs the appropriate ROS 2 installer (`ros-humble-install.sh` or `ros-jazzy-install.sh`).
	After the ROS installation completes, the script will proceed to install the Micro‑ROS agent if it is not already present.

3. Install the UDMarsRover packages with the setup script:
    ```bash
    ./setup.sh
    ```

    This will build the ROS packages in the workspace and source the setup files.

## Running the Rover

Use the provided `run.sh` script to start all required ROS nodes:

```bash
cd ~/UDMarsRover/rover_setup
./run.sh
```

This will launch:

* `micro_ros_agent` connected to the rover over a serial port.
* `twist_control` and `joy_twist` for converting joystick input to motion commands.
* `joint_state_pub` for wheel/joint feedback.
* `joy_node` for reading the game controller.

### Serial Port Configuration

By default the agent connects to `/dev/ttyACM0`. If your board enumerates at a different path, edit `src/rover/launch/rover_launch.py`:

```python
Node(
	 package='micro_ros_agent',
	 executable='micro_ros_agent',
	 arguments=['serial', '--dev', '/dev/ttyUSB1'],
)
```

Alternatively, you can create a udev rule or use a symlink for a stable name.

### Controller Requirement

The rover is teleoperated via a joystick. Make sure a supported controller is plugged into the computer before running `run.sh`. Without a controller, the `joy_node` will fail and you will not be able to drive the rover.

## Troubleshooting

- If `run.sh` exits immediately, check the ROS environment variables or re-source the setup files.
- If the micro ROS agent fails to communicate, verify the serial port (see above) and check permissions (`sudo usermod -aG dialout $USER`).
- If the joystick is not detected, run `jstest-gtk` or similar to verify OS-level recognition.

Enjoy exploring with your mini rover!

