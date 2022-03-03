[![ci](https://github.com/HarvestX/MG400_ROS2/actions/workflows/ci.yml/badge.svg)](https://github.com/HarvestX/MG400_ROS2/actions/workflows/ci.yml)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# MG400_ROS2
Dobot MG400 ROS2 Repository.

![Image](https://github.com/HarvestX/MG400_ROS2/blob/main/media/display.png?raw=true)


## Requirements
- Linux OS
  - [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- ROS 2
  - [Galactic Geochelone](https://index.ros.org/doc/ros2/Installation/Galactic/)


## Install
### Locate package in workspace
```bash
mkdir -p ~/ws_galactic/src
cd ~/ws_galactic/src
git clone git@github.com:HarvestX/MG400_ROS2.git
```

### Run script to install dependencies
```bash
source /opt/ros/galactic/setup.bash
cd ~/galactic_ws/src/MG400_ROS2
./setup.bash
exec -l $SHELL
```

## Build Source
Open new terminal and type followings.
```bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws
colcon build
```

## Launch Ros2 System
Before launch MG400 scripts, open new terminal and type followings.
```bash
source ~/ws_galactic/install/setup.bash
```

### Launch display (It works without hardware.)
```bash
ros2 launch mg400_bringup display.launch
```

### Launch main system (MG400 should be connected via LAN cable.)
```bash
ros2 launch mg400_bringup mg400.launch
```

## References
### Official ROS1 package
- [Official ROS1 package](https://github.com/Dobot-Arm/MG400_ROS)
- [Connection trouble shouting](https://drive.google.com/file/d/1XZdcXGPddbkGDYDBaovpLm1Mz8kck3Tj/view)
