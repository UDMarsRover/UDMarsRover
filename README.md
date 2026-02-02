# UDMarsRover
This is the primary repository for the University of Dayton Mars Rover team's rover. In this repository, you will find all code that runs on UDMRT systems at competition.

## Requesting Access
To gain access to the UDMarsRover repository, please complete [this form](https://forms.gle/PrTdcqrjhV6DZBuG8).

## Execution

Install ROS dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace

```bash
colcon build
``` 

Source the workspace

```bash
source install/setup.bash
```

TODO: Launch base or rover systems.
