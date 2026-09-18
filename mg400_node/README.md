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

### State topics

- `robot_mode`: Raw RobotMode value from realtime feedback.
- `robot_state`: Normalized read-only state with the last raw RobotMode and feedback freshness.

The `robot_state` topic uses reliable, transient-local, keep-last-one QoS so a
late subscriber receives the latest state. When realtime feedback becomes stale
or the interface disconnects, `state` becomes `UNKNOWN` and `feedback_fresh`
becomes `false` while `raw_robot_mode` retains the last valid value.

## Joint State Publisher Gui

Start joint state publisher GUI.

```bash
ros2 run mg400_node joint_state_publisher_gui
```
