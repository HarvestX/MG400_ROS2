# MG400 RViz panels

`mg400_rviz_plugin/Mg400Controller` provides one panel with shared **Enable**,
**Disable**, and **Clear Error** buttons at the top and **MovJ / JointMovJ / Collision** tabs
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

1. Use **Clear Error** when needed. Enter **Load [kg]** and **Center X/Y/Z [mm]**,
   then click **Enable** to apply the payload and enable the robot.
   The allowed ranges match Identify: 0–0.75 kg and ±500 mm, with up to three
   decimals. Every Enable sends all four values together using `FOUR_PARAM`.
   The fields start at zero; empty or invalid values prevent enabling.
   Payload fields can be edited before enabling and are locked while enabled or
   a command is pending. These inputs are not saved in the RViz configuration.
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

Use the **Collision** tab to select a **Target level** from **0 (Off)** to **5**,
then click **Apply Collision Level**. Level 0 disables collision detection.
No level is selected initially; selecting a level does not send it automatically.
Applying requires robot mode `DISABLED` or `ENABLE`, an available service, and no
pending motion or service request. The selector is locked during a request or motion.
The shared service status above the tabs reports the requested level and success or
failure/error ID, or a timeout after five seconds. This is the service result, not
a readback of the robot's current collision level. The selection is not saved in
the RViz configuration.

The panel uses `/mg400/enable_robot`, `/mg400/disable_robot`,
`/mg400/clear_error`, `/mg400/set_collision_level`, `/mg400/mov_j`, and `/mg400/joint_mov_j`.
Current values come from `/mg400/joint_states` and action feedback. Physical
J1–J4 map to `mg400_j1`, `mg400_j2_1`, `mg400_j4_2`, and `mg400_j5`, respectively.
Reordered joint names and a joint-name prefix are supported.

## Identify panel

`mg400_rviz_plugin/Identify` is independent of `Mg400Controller`. It loads an
experiment YAML and performs the complete move → settle → record sequence.

```bash
ros2 launch mg400_bringup param_identify.launch.py ip_address:=192.168.1.6
```

The driver publishes raw joint currents and end poses with force estimation
disabled during collection. `namespace` defaults to `mg400`; services, actions,
and telemetry are remapped together. The panel starts with no experiment loaded.
An example YAML is available at [`mg400_tools/config/param_identify.yaml`](../mg400_tools/config/param_identify.yaml)
and is installed under `share/mg400_tools/config`. Copy it to a writable location
before editing:

```bash
cp "$(ros2 pkg prefix --share mg400_tools)/config/param_identify.yaml" ./experiment.yaml
```

For an already running driver publishing currents, use
`ros2 launch mg400_bringup rviz.launch.py rviz_config:=param_identify.rviz`.

1. Use **Load YAML...** while disabled to select the experiment. Review its path,
   payload, motion/measurement settings, and pose table.
2. Click **Enable**. The panel sends all four payload parameters using `FOUR_PARAM`.
3. Click **Run and record...** and choose an `.mcap` filename. If either the MCAP or its matching YAML already exists,
   confirm **Yes** to replace it and run; **Cancel** keeps the file and sends no motion.
   Recording starts inside RViz before the first move; no separate recorder terminal
   is needed. The panel executes the complete pose list with explicit
   speed/acceleration and CP=0.
4. Repeat Run with new output names, or use **Stop / Disable** to abort and save.

All settings are read-only in RViz. Edit the YAML, then Disable → Load YAML →
Enable to apply changes. Loading never enables or moves the robot. Editing the
file on disk does not change an already loaded experiment. Experiment settings
and their YAML path are not saved in the RViz configuration. After restarting
RViz, select the experiment again using **Load YAML...**.

The YAML contains `payload`, `motion`, `measurement`, `identify`, and `poses`.
Units are part of key names: `load_kg`, `center_x_m`, `speed_percent`,
`acceleration_percent`, `settle_sec`, `record_sec`, `torque_constants_nm_per_a`,
`reference_deg`, `min_duration_sec`, `max_span_deg`, and `joints_deg`.
Payload coordinates may alternatively use `center_x_mm / center_y_mm / center_z_mm`;
all three must use the same unit. The default load is 0.1 kg and center is
0.045/0.012/0.01 m, converted to 45/12/10 mm for Enable. Payload range is 0–0.75 kg
and ±500 mm, with at most three decimals after conversion to kg/mm.
Unknown, missing, duplicate, non-finite, or out-of-range settings are rejected.

| Kind | Purpose |
| --- | --- |
| `base` | Measure current bias at `reference_deg`. |
| `train` | Fit posture compensation coefficients. |
| `check` | Measure held-out residuals without affecting the fit. |
| `move` | Move and settle without a measurement interval. |

Each pose uses `{kind: train, joints_deg: [10, 40, 50, 15]}`. The supplied YAML
contains 40 training poses, 12 checking poses, and 7 baseline measurements.
Review poses and connecting paths for the tooling and surroundings before Run.
Joint-limit validation includes J2/J3 coupling; it does not perform collision planning.

Run requires this panel's successful Enable and fresh state, angles, and currents.
The target tolerance is 0.5°, and the stationary anchor tolerance is 0.2°.
Motion failure, stale state/telemetry, or movement during sampling aborts the run
and requests Disable. Action cancellation alone does not stop the current JointMovJ
server. Check the Disable result; communication loss can prevent stopping.
Completed runs leave the robot enabled at the final pose. Use this panel as the
only motion/Enable client during collection.

### Recording and offline fitting

The default recording basename is `param_identify_<timestamp>`. For example,
choosing `run.mcap` writes that file using `rosbag2_cpp` and the MCAP storage plugin,
plus `run.yaml` in the same directory. The files are paired by basename only;
there is no hash comparison. The YAML preserves the loaded experiment settings
(including torque constants) and adds a `recording` section. The previous output
pair remains intact during a replacement run and is replaced after recording closes.

The bag contains only the original `joint_states`, `joint_currents` (actual and
target amperes), and `robot_mode` messages under their resolved topic names.
No additional ROS message definitions or annotation topics are used.
Original `header.stamp` values are preserved; bag timestamps represent receipt time.
Physical J1–J4 map to `mg400_j1`, `mg400_j2_1`, `mg400_j4_2`, and `mg400_j5`,
including prefixed/reordered names.

The YAML's `recording` section contains the recording start/end times, result,
resolved topic names, confirmed Enable payload, and `windows`. Each window records
its segment ID, kind, goal angles, start/end timestamps (`sec`, `nanosec`), and
`complete` flag. Windows use the same ROS clock as the bag's receipt timestamps.
The reader selects messages in `[start, end)` by receipt time, then uses their
original source timestamps for stream alignment and statistics. Buffered samples
produced before the window are excluded. Only completed windows enter fitting;
interrupted windows are excluded. Enable payload describes this panel's successful
request, not controller readback. Saved YAML can also be loaded for another Run;
the old `recording` section is replaced.

After building `mg400_tools` and sourcing the ROS workspace, run the offline
analysis from any working directory. The tools package declares its dependencies
on `rosbag2_py`, `rosbag2_storage_mcap`, NumPy, and PyYAML.
No robot or running ROS nodes are needed. Use the path to your recording if it is
stored elsewhere.

```bash
ros2 run mg400_tools param_identify run.mcap \
  --output identified.yaml
```

The script reads each MCAP's same-stem YAML and uses its `identify` settings.
Keep both files together when copying or renaming a recording. All identification
settings come from YAML; there are no per-setting CLI overrides. Multiple runs
must use compatible torque constants, signs, reference angles, and payloads;
their pose lists, comments, and motion settings may differ.
The output contains only `joint_current_bias_a` (four values) and
`posture_coefficients_nm` (44 values, 11 per joint in J1–J4 order).
The console prints only the output path. Torque gains, friction, and inertial
parameters are not fitted. The driver does not automatically load this result YAML.
To apply it, map `joint_current_bias_a` to `ExternalForceEstimator::Config::joint_current_bias`
and `posture_coefficients_nm` to `ExternalForceEstimator::Config::posture_coefficients`,
using the same torque constants and signs as the identification settings.

The CLI accepts recording MCAPs, `--output`, and optional `--force` (plus `--help`).
Each input requires the matching YAML with experiment settings and measurement
windows. Existing CSV recordings and bags without this accompanying YAML are not
accepted. Input MCAPs and their YAML cannot be overwritten; other existing outputs
require `--force`.

MCAP files are finalized on completion, Stop, or normal panel close. Active data
is staged in a `.param_identify-*` directory beside the chosen output. On a finalization
error, the panel reports and retains that directory for recovery; an abrupt crash
can leave an unfinished bag. The script checks overlapping stream duration, gaps,
mode, stationarity, payload consistency, and fit rank before writing coefficients.

## Tests without hardware

```bash
colcon test --packages-select mg400_rviz_plugin
colcon test-result --verbose
```

Tests use Qt's offscreen backend and local mock action/service servers in ROS
domains 187 (controller) and 188 (identify). They do not connect to the robot.
The recording-to-analysis integration test imports `mg400_tools`, which is a test
dependency. The Python analysis tests belong to `mg400_tools/test`.
