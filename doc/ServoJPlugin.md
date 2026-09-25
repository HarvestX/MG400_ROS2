# ServoJ Motion API plugin

`mg400_plugin::ServoJ` streams four-joint targets to an enabled MG400.
It is in `mg400_node`'s default `motion_api_plugins` list. If that list
is overridden, include `mg400_plugin::ServoJ` to expose these interfaces.
The plugin accepts sessions only while the node is connected to the robot.

## ROS interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `servo_j_session` | `mg400_msgs/srv/ServoJSession` | Start or stop one session |
| `servo_j` | `mg400_msgs/msg/ServoJ` | Publish joint targets for that session |
| `robot_state` | `mg400_msgs/msg/RobotState` | Check robot state and feedback freshness |

The node publishes `robot_state`; the plugin provides the first two
interfaces. Names are relative to the node namespace. With the default
`ros2 launch mg400_bringup main.launch.py`, they resolve under `/mg400/`.

## Configuration

`servo_j_default_t` is the only ServoJ node parameter. Its default is
`0.1` seconds; its valid range is `[0.004, 3600]` seconds. The plugin
reads it when configured and uses it when a setpoint has `t: 0.0`.

## Operating sequence

1. Connect and enable the robot. Confirm `robot_state.feedback_fresh: true`
   and `robot_state.state: 4` (`ENABLED`); no ServoJ session may be active.
2. Call `servo_j_session` with `start: true` and `session_id: 0`.
   Proceed only if `success: true`, and retain the returned `session_id`.
3. Publish `ServoJ` messages with that ID, four `joint_positions` in
   radians, and a point execution time `t` in seconds. Begin at the
   current controller joint angles, then send speed-planned points at
   approximately 30 Hz. Do not jump directly to a distant target.
4. Stop publishing and call `servo_j_session` with `start: false` and
   the same ID. A successful response releases the session. If it fails,
   wait for the stated condition and retry the stop request.

`joint_positions` is ordered J1, J2, J3, J4. J3 is the controller's
absolute joint angle, **not** the relative ROS `j3_1` angle. To derive
controller angles from `joint_states`, use J1 = `j1`, J2 = `j2_1`,
J3 = `j2_1 + j3_1`, and J4 = `j5` (with any configured joint-name prefix).
Setpoints outside MG400 joint limits, with non-finite angles, or with an
invalid `t` are ignored. A nonzero `t` must be finite and within
`[0.004, 3600]` seconds. The wire command is
`ServoJ(J1,J2,J3,J4,t=value)`; the plugin converts radians to degrees.

## Session and timing behavior

- The setpoint subscription uses best-effort, volatile, keep-last-one
  QoS. A 30 ms wall timer sends the latest **new** valid setpoint once;
  superseded points are not queued or replayed.
- If no setpoint arrives for more than 150 ms, sending pauses but the
  session remains active. A new valid setpoint can resume the stream.
- While a session is active, `robot_state.state` is `8` (`SERVO`)
  when raw feedback is `ENABLE` or `RUNNING`. Other motion plugins
  check this state before accepting commands. This is not an atomic
  lock across all motion commands or external robot controls.
- A stop request ceases accepting setpoints immediately. It succeeds
  only at least 300 ms after the last sent point, with a connected
  robot and fresh raw `ENABLE` feedback. Retry after a failed stop;
  publishing more setpoints will not resume a stopping session.
- Disconnection, stale feedback, or a robot mode other than `ENABLE`
  or `RUNNING` invalidates the session. After recovery, start a new
  session and use its new ID.
- TCP transmission is synchronous. The 30 ms timer is a send
  opportunity, not a guaranteed 30 Hz delivery rate.
