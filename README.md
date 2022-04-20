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

## Connect to MG400
Connect PC and MG400 to the same Ethernet via LAN cable.
MG400 needs the emergency stop switch.

### IP address settings
Set the PC's Ethernet IPv4 IP address to a fixed IP somewhere on 192.168.1.0/24 other than 192.168.1.6
(Example: 192.168.1.10, See https://linuxfan.info/ubuntu-2004-desktop-static-ip-address)
No need to fill in the "Gateway" and "DNS".

![Image](https://github.com/HarvestX/MG400_ROS2/blob/main/media/IPv4_settings.png?raw=true)

Check the circle light on the base of MG400 blink blue.

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

### Test the sample program
Launch main system with other terminal.
Allow enough space around the robot as it moves.
```bash
. sample.bash
```

## Running Utilities
### TCP Packet Checker
```bash
ros2 run mg400_control realtime_packet_checker
```

## References
### Official ROS1 package
- [Official ROS1 package](https://github.com/Dobot-Arm/MG400_ROS)
- [Connection trouble shooting](https://drive.google.com/file/d/1XZdcXGPddbkGDYDBaovpLm1Mz8kck3Tj/view)
