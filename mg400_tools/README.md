# MG400 tools

This package contains C++ and Python command-line tools for MG400 operation
and kinematics checks. Source the ROS workspace after building it, then run
commands from any working directory with `ros2 run`.

| Command | Purpose |
| --- | --- |
| `command_sender` | Replay command logs through the robot's TCP interfaces; supports `--dry-run`. |
| `compare_positive_inverse_solution` | Check the robot's PositiveSolution/InverseSolution TCP responses. |
| `command_queue_client` | Send the configured command sequence through the ROS command queue action. |
| `validate_kinematics_with_solutions` | Compare local kinematics with the robot's ROS solution services. |
| `mg400_kinematics_cli` | Evaluate local forward or inverse kinematics. |

## Usage

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

## Source layout

- `src/` and `include/mg400_tools/`: C++ tools.
- `mg400_tools/`: importable Python implementations.
- `scripts/`: small Python command entry points installed under `lib/mg400_tools`.
- `config/` and `examples/`: resources installed under `share/mg400_tools`.

The mixed C++/Python package uses `ament_cmake_python` to install its Python module.
