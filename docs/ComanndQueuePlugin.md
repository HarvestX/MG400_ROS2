# CommandQueue Plugin

The `CommandQueue` plugin provides an action-based interface for executing a sequence of motion commands on the Dobot MG400 robot via ROS 2. It is implemented as a plugin for the [`mg400_plugin_base`](../../mg400_plugin_base/) framework.

## Features

- Accepts a queue of motion commands (`JointMovJ`, `MovJ`, `MovL`, `MovJIO`, `MovLIO`) via a ROS 2 action interface.

## Action Interface

The plugin exposes the `/mg400/command_queue` action server using the [`mg400_msgs/action/CommandQueue`](../mg400_msgs/action/CommandQueue.action) definition.

### Goal

A goal consists of a sequence of motion commands, each specifying a type and parameters (see [`mg400_msgs/msg/Command`](../mg400_msgs/msg/Command.msg)).

### Feedback

Feedback includes the current pose and joint angles of the robot during execution.

### Result

The result indicates success or failure, and includes error IDs.

## Extending

- The main logic is implemented in [`command_queue.cpp`](../mg400_plugin/src/motion_api/command_queue.cpp) and [`command_queue.hpp`](../mg400_plugin/include/mg400_plugin/motion_api/command_queue.hpp).

- To add new command types or customize validation, extend the `sendCommand` and `validateTarget` methods in [`mg400_plugin::CommandQueue`](../mg400_plugin/include/mg400_plugin/motion_api/command_queue.hpp).

---

# CommandQueueClient Tool

The `command_queue_client` node (provided by the `mg400_tools` package) is a ROS 2 node developed as a test tool for the `CommandQueue` plugin. It sends a sequence of motion commands to the MG400 using the command_queue action interface.

## Features

- Loads a list of commands from a YAML configuration file.
- Supports all command types: `JointMovJ`, `MovJ`, `MovL`, `MovJIO`, `MovLIO`.
- Provides detailed feedback and result logging, including error IDs.

## Configuration

The client loads a YAML file named `<install_share>/mg400_tools/config/commands.yaml`, which defines the sequence of commands.

A sample `commands.yaml` might look like:

```yaml
commands:
  - type: "MovJ"
    pos_m: [0.3, 0.0, 0.2]
    yaw_deg: 0.0
    speed_j: 20
    acc_j: 20
    cp: 0
  - type: "JointMovJ"
    angles_deg: [0, -45, 90, 0]
    speed_j: 20
    acc_j: 20
    cp: 0
```

## Running the Client

After launching the robot system, run:

```bash
source install/setup.bash
ros2 service call /mg400/enable_robot mg400_msgs/srv/EnableRobot "{}"
ros2 run mg400_tools command_queue_client --ros-args --remap __ns:=/mg400
ros2 service call /mg400/disable_robot mg400_msgs/srv/DisableRobot "{}"
```

The node automatically read the configuration, send the commands, and print feedback and results to the console.

## Extending

To add new command types or customize YAML parsing, extend the parsing methods in [`command_queue_client.cpp`](../mg400_tools/src/command_queue_client.cpp).
