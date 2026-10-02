# MG400 Bringup

ROS2 package for launch files.

## Launch service server

The command below lists all arguments declared by `main.launch.py` with their
default values. All arguments are optional; change the values you need.

```bash
ros2 launch mg400_bringup main.launch.py \
    namespace:=mg400 \
    ip_address:=192.168.1.6 \
    joy:=false \
    enable_external_force_estimator:=false \
    publish_end_pose:=false \
    publish_joint_currents:=false \
    publish_error_id:=false \
    workspace_visible:=False
```

To display launch arguments and their descriptions without starting the nodes:

```bash
ros2 launch mg400_bringup main.launch.py --show-args
```

## Connect launch server with MG400_Mock

Check the ip address of MG400 Mock.
https://github.com/HarvestX/MG400_Mock#identify-container-ip-address

```bash
ros2 launch mg400_bringup main.launch.py ip_address:=<ip_address of mock>
```
