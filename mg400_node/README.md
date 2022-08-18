# MG400_node

## Service Server Node

Start service server to operate MG400 via ROS2 service.

```bash
ros2 run mg400_node service_node_exec
```

### Available Services

#### service level 1

- `clear_error`
- `disable_robot`
- `enable_robot`
- `move_jog`
- `mov_j`

#### service level 2

- `reset_robot`
- `speed_factor`
- `joint_mov_j`
- `mov_l`

#### service level 3

- `tool_do_execute`

## Joint State Publisher Gui

Start joint state publisher gui.

```bash
ros2 run mg400_node joint_state_publisher_gui
```
