# MG400_node

## API Interface Node
Start API interface to operate MG400 via ROS2 service/action.

```bash
ros2 run mg400_node mg400_node_exec
```

### Automatic error ID publishing

`publish_error_id` (bool, default: `false`) controls the periodic `GetErrorID()`
query, error-detail logging, and `error_id` publishing while the robot is in ERROR mode.
Automatic polling is disabled by default. Set it to `true` to enable it:

```bash
ros2 run mg400_node mg400_node_exec --ros-args -p publish_error_id:=true
```

It can also be changed at runtime (replace the node path if using another namespace):

```bash
ros2 param set /mg400/mg400_node publish_error_id false
ros2 param set /mg400/mg400_node publish_error_id true
```

The next error timer callback uses the new value; its interval is 500 ms.
The `robot_mode` topic and explicit `get_error_id` service requests remain available.

### Default API plugin for `mg400_node`
The following API interface plugin will be loaded by default
- Dashboard API
  - `ClearError`
  - `DisableRobot`
  - `EmergencyStop`
  - `EnableRobot`
  - `GetErrorID`
  - `InverseSolution`
  - `PayLoad`
  - `PositiveSolution`
  - `ResetRobot`
  - `SetCollisionLevel`
  - `SpeedFactor`
  - `ToolDOExecute`
  - `ToolDI`
- Motion API
  - `CommandQueue` (detailed [here](../doc/ComanndQueue.md))
  - `JointMovJ`
  - `MoveJog`
  - `MovJ`
  - `MovJIO`
  - `MovL`
  - `MovLIO`

## Joint State Publisher Gui

Start joint state publisher GUI.

```bash
ros2 run mg400_node joint_state_publisher_gui
```
