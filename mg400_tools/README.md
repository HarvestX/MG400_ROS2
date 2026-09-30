# MG400 tools

This package contains C++ and Python command-line tools for MG400 operation,
kinematics checks and offline parameter identification. Source the ROS workspace
after building it, then run commands from any working directory with `ros2 run`.

| Command | Purpose |
| --- | --- |
| `param_identify` | Fit current bias and posture compensation from MCAP and YAML recordings. |
| `command_sender` | Replay command logs through the robot's TCP interfaces; supports `--dry-run`. |
| `compare_positive_inverse_solution` | Check the robot's PositiveSolution/InverseSolution TCP responses. |
| `command_queue_client` | Send the configured command sequence through the ROS command queue action. |
| `validate_kinematics_with_solutions` | Compare local kinematics with the robot's ROS solution services. |
| `mg400_kinematics_cli` | Evaluate local forward or inverse kinematics. |

## Parameter identification

Copy the experiment template to a writable location and edit it before loading it
in the RViz Identify panel:

```bash
cp "$(ros2 pkg prefix --share mg400_tools)/config/param_identify.yaml" ./experiment.yaml
ros2 launch mg400_bringup param_identify.launch.py ip_address:=192.168.1.6
```

The panel's **Run and record...** command defaults to the basename
`param_identify_<timestamp>`. Choosing `run.mcap` creates `run.mcap` and `run.yaml`.
Keep the files together with matching basenames. The MCAP contains raw telemetry;
the YAML contains the experiment settings and timestamped measurement windows.
See the [Identify panel workflow](../mg400_rviz_plugin/README.md#identify-panel)
for collection details.

```bash
ros2 run mg400_tools param_identify run.mcap --output identified.yaml
```

Each input MCAP needs its own matching YAML. Multiple recordings can be supplied
before `--output`. The result contains `joint_current_bias_a` (four values) and
`posture_coefficients_nm` (44 values in joint-major order). Use `--force` to replace
an existing result. Input MCAPs and their YAML are protected from overwriting.
The analysis works offline and does not communicate with the robot.

The Python implementation can also be imported:

```python
from mg400_tools.param_identify import read_recording
```

## Other tools

Inspect a recorded command list without sending commands:

```bash
ros2 run mg400_tools command_sender \
  --input "$(ros2 pkg prefix --share mg400_tools)/examples/command_list.txt" \
  --dry-run
```

`config/commands.yaml` configures the C++ `command_queue_client`; the TCP
`command_sender` reads text command logs. These tools use different interfaces
and input formats. See the [command queue documentation](../doc/ComanndQueuePlugin.md).

Evaluate local forward kinematics, with joint angles in degrees:

```bash
ros2 run mg400_tools mg400_kinematics_cli fk 0 10 20 30
```

URDF generation is provided by the robot description package:

```bash
ros2 run mg400_description generate_urdf mg400
```

This writes `mg400.urdf` in the current directory.

## Source layout and tests

- `src/` and `include/mg400_tools/`: C++ tools.
- `mg400_tools/`: importable Python implementations.
- `scripts/`: small Python command entry points installed under `lib/mg400_tools`.
- `config/` and `examples/`: resources installed under `share/mg400_tools`.
- `test/`: offline analysis tests.

The mixed C++/Python package uses `ament_cmake_python` to install its Python module.
Its tests import the installed module and require a built, sourced workspace.

```bash
colcon test --packages-select mg400_tools
colcon test-result --verbose
```
