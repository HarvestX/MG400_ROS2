# MG400 Bringup
ROS2 package for launch files.

## Launch service server
```bash
ros2 launch mg400_bringup main.launch.py
```

## Connect launch server with MG400_Mock
```bash
ros2 launch mg400_bringup main.launch.py ip_address:=<ip_address of mock>
```
