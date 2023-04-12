# MG400_node

## API Interface Node
Start API interface to operate MG400 via ROS2 service/action.

```bash
ros2 run mg400_node mg400_node_exec
```
### Default API plugin for `mg400_node`
The following API interface plugin will be loaded by default
- Dashboard API
  - `ClearError`
  - `DisableRobot`
  - `EnableRobot`
  - `ResetRobot`
  - `SpeedFactor`
  - `ToolDOExecute`
- Motion API
  - `MoveJog`
  - `MovJ`

## Joint State Publisher Gui

Start joint state publisher GUI.

```bash
ros2 run mg400_node joint_state_publisher_gui
```
