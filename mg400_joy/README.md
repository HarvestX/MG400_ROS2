# mg400_joy

Dobot MG400 ROS2 ServoMode controller with a game controller.

## Launch

```bash
ros2 launch mg400_bringup joy.launch.py
```

This launch file starts only the `joy` node and `mg400_joy_interface_node`.
It does not start `mg400_node`, robot state publishing, or RViz.

To run the MG400 node, robot description, RViz, and joystick control together,
use `main.launch.py` with `joy:=true`:

```bash
ros2 launch mg400_bringup main.launch.py joy:=true
```

For a real robot, pass the MG400 IP address to `main.launch.py`:

```bash
ros2 launch mg400_bringup main.launch.py joy:=true ip_address:=192.168.1.6
```

Mapping file and service timeout arguments can also be passed through
`main.launch.py` when `joy:=true`:

```bash
ros2 launch mg400_bringup main.launch.py \
  joy:=true \
  mapping_file:=$(ros2 pkg prefix --share mg400_joy)/config/dualshock.yaml \
  service_timeout_ms:=5000
```

By default, no mapping file is used. The node learns the raw Joy mapping from
the controller inputs during startup. To use the included DualShock/DualSense
mapping file instead:

```bash
ros2 launch mg400_bringup joy.launch.py \
  mapping_file:=$(ros2 pkg prefix --share mg400_joy)/config/dualshock.yaml
```

## Description

`mg400_joy` converts `sensor_msgs/msg/Joy` input from the `joy` package into
MG400 ServoMode commands. It reads raw `buttons[]` and `axes[]` indexes directly
and does not depend on `p9n_interface`.
It enters and exits ServoMode through the `servo_mode` service, then publishes
continuous `servo_j` or `servo_p` targets while ServoMode is active.

While running, it publishes a `joy_control_active` heartbeat in the robot
namespace. The RViz Servo panel uses that heartbeat to disable its own ServoMode
controls and avoid publishing competing `servo_j` / `servo_p` targets.

The target starts from the latest `realtime_feedback` state. Stick input is
integrated into the target at the configured speed. If fresh `joy` messages stop
arriving, this node stops publishing servo commands and lets the MG400 node's
ServoMode timeout stop the session.

## Startup Controller Check

By default, the node learns the raw Joy mapping before it accepts robot-control
inputs. Move or press only the prompted input. If multiple inputs are active at
once, joy control is locked. If two prompts resolve to the same button or axis
index, calibration is rejected.

The log prompts the user to:

1. Press `Cross/A`.
2. Release all inputs.
3. Press `L1/LB`.
4. Release all inputs.
5. Press `R1/RB`.
6. Release all inputs.
7. Press `Select/Back`.
8. Release all inputs.
9. Press `Start/Menu`.
10. Release all inputs.
11. Press `PS/Guide`.
12. Release all inputs.
13. Move the left stick right.
14. Release all inputs.
15. Move the left stick up.
16. Release all inputs.
17. Move the right stick right.
18. Release all inputs.
19. Move the right stick up.
20. Release all inputs.
21. Press `D-pad Up`.

## Buttons

| Target | Command | Description |
| ------ | ------- | ----------- |
| Cross/A, L1/LB, R1/RB, Select/Back, Start/Menu, PS/Guide, sticks, D-pad Up | mapping calibration | learned during startup |
| Start | `EnableRobot` or `ServoMode(true/false)` | enable when disabled, enter when enabled, exit when active |
| Select | `DisableRobot` or `ServoMode(false)` | disable when enabled, exit when active |
| PS | `ServoMode(false)` + `ResetRobot` | exit ServoMode before reset |
| R1 | switch `ServoJ` / `ServoP` | only while ServoMode is inactive |

## ServoJ Control

Default mode is `ServoJ`.

| Target | Servo target |
| ------ | ------------ |
| Left Stick left | J1 + |
| Left Stick right | J1 - |
| Left Stick up | J2 - |
| Left Stick down | J2 + |
| Right Stick up | J3 + |
| Right Stick down | J3 - |
| Right Stick left | J4 + |
| Right Stick right | J4 - |

## ServoP Control

Switch to `ServoP` with `R1` before entering ServoMode.

| Target | Servo target |
| ------ | ------------ |
| Left Stick up | X + |
| Left Stick down | X - |
| Left Stick left | Y + |
| Left Stick right | Y - |
| Right Stick up | Z + |
| Right Stick down | Z - |
| Right Stick left | Rx + |
| Right Stick right | Rx - |

## Parameters

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `mapping_file` | empty | Optional YAML raw Joy mapping file. Empty means learn mapping at startup |
| `servo_control_type` | `ServoJ` | Initial control type: `ServoJ` or `ServoP` |
| `linear_speed_mm_s` | `20.0` | ServoP XYZ target speed |
| `angular_speed_deg_s` | `15.0` | ServoP Rx target speed |
| `joint_speed_deg_s` | `15.0` | ServoJ target speed |
| `stick_deadzone` | `0.05` | Stick values below this are treated as zero |
| `controller_axis_threshold` | `0.5` | Axis threshold used for controller check inputs |
| `servo_publish_period_ms` | `15` | Servo command publish period |
| `joy_timeout_ms` | `250` | Stop publishing when joy input is stale |
| `service_timeout_ms` | `5000` | Service wait and response timeout |
| `controller_check_timeout_sec` | `60.0` | Startup controller check timeout |

## Mapping File

The `joy` node still publishes the raw `sensor_msgs/msg/Joy` message on
`/mg400/joy`. A mapping file can provide the raw indexes directly and skip
startup learning.

The first received `Joy` message is recorded as the neutral value for the analog
sticks. Servo commands use stick values relative to that neutral baseline and
normalize the prompted right/up stick directions as positive raw directions.
Buttons are checked as raw pressed inputs. D-pad Up can be a button or an axis
delta from neutral. Button indexes must be unique, stick axis indexes must be
unique, and D-pad Up must not overlap with either group.

```yaml
buttons:
  cross: 0
  l1: 4
  r1: 5
  select: 8
  start: 9
  ps: 10
axes:
  stick_lx:
    index: 0
    positive_value: -1.0
  stick_ly:
    index: 1
    positive_value: -1.0
  stick_rx:
    index: 3
    positive_value: -1.0
  stick_ry:
    index: 4
    positive_value: -1.0
dpad_up:
  axis: 7
  axis_value: 1.0
```
