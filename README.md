# dadsim
Meta-package of DADSim

## Install

1. Install ROS2. ROS Galactic is the tested version.
2. Install dependencies with rosdep. The package `ackermann_msgs` may not be able to be installed automaticly. Install it with `apt install ros-VERSION-ackermann-msgs` (replace VERSION with your specific ros version, e.g. galactic).
3. Clone this repository.
4. Update all submodules:
```shell
git submodule update --init --recursive
```
5. Build them with
```shell
colcon build
```
6. Enjoy!

## Start-up

Launch the test file:
```shell
ros2 launch dadsim test.launch.py
```
Open RViz and add following topics:
- /map_visualize
- /agents
- /agents_array
You will see 10 white cars and 1 green car. You can control the green one by:
```shell
ros2 run dadsim ackermann_teleop
```
Use W/A/S/D to drive it!