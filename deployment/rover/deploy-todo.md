# Rover Deployment Necessities

## Installation

- Install and activate a systemd service to run the rover container on boot.
- Place the docker executables in the correct directory.
- Setup a udev rule to assign a static name to the rover's USB device.

## Execution Config
Things the docker container needs to run:
- Install rover package and dependencies.
- Launch the rover launch file