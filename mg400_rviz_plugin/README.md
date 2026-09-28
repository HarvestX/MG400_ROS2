# MG400 RViz control panel

`mg400_rviz_plugin/Mg400Controller` provides one panel with shared **Enable**,
**Disable**, and **Clear Error** buttons at the top and **MovJ / JointMovJ** tabs
below them. The default `mg400.rviz` configuration includes this panel.

For an existing RViz configuration, use **Panels → Add New Panel →
mg400_rviz_plugin → Mg400Controller**. Remove the old separate
`Mg400JointController` panel if it is present in a previously saved configuration.

## Build and launch

Run from your ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mg400_rviz_plugin mg400_bringup
source install/setup.bash
ros2 launch mg400_bringup main.launch.py
```

If the robot driver is already running, launch only RViz:

```bash
ros2 launch mg400_bringup rviz.launch.py
```

## Operation

1. Use **Clear Error** when needed, then **Enable** using the shared top buttons.
2. Select a tab:
   - **MovJ**: enter X/Y/Z in **mm** and R in **degree**. **Use Current** copies
     the current pose into all four target fields.
   - **JointMovJ**: enter J1–J4 in **degree**. **Use Current** copies the current
     joint angles into all four target fields.
3. Edit the target and click **Send MovJ** or **Send JointMovJ**.

Switching tabs preserves the entered targets and never sends a command.
While either motion is pending, both tabs prevent further sends. The shared
**Disable** button remains available during movement when its service is ready.
Robot service calls run asynchronously and report success, failure/error ID, or
no response after five seconds. Motion results are shown below the tabs.

Targets start empty. Sending requires valid numeric values, robot mode `ENABLE`,
and an available action server. JointMovJ targets are also checked with the
server's joint range validation, including the coupled J2/J3 limit.
MovJ targets use `mg400_origin_link`. Positions are converted from mm to meters,
and angles from degrees to radians for the existing action interfaces.
Speed, acceleration, and CP overrides are left unset, so the robot's existing
settings apply.

The panel uses `/mg400/enable_robot`, `/mg400/disable_robot`,
`/mg400/clear_error`, `/mg400/mov_j`, and `/mg400/joint_mov_j`.
Current values come from `/mg400/joint_states` and action feedback. Physical
J1–J4 map to `mg400_j1`, `mg400_j2_1`, `mg400_j4_2`, and `mg400_j5`, respectively.
Reordered joint names and a joint-name prefix are supported.

## Tests without hardware

```bash
colcon test --packages-select mg400_rviz_plugin
colcon test-result --verbose
```

Tests use Qt's offscreen backend and local mock action/service servers in ROS
domain 187. They do not connect to the robot.
