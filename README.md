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