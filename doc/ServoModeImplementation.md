# ServoJ implementation guide

This document maps the ServoJ behavior to the repository implementation.

## Package responsibilities

| Package | Responsibility |
| --- | --- |
| `mg400_msgs` | Servo-state, target, error, and service definitions |
| `mg400_interface` | ROS-independent lease arbitration, safety validation, periodic transmission, response monitoring, and safe stop |
| `mg400_node` | Lifecycle integration and ROS services, subscriptions, and publishers |
| `mg400_operation_gui` | Optional interactive ServoJ client |

## Main classes

### `ControlStateManager`

`ControlStateManager` is the thread-safe motion-ownership arbiter. It tracks the
controller connection, RobotMode, ROS control state, lease ID, and whether
Servo targets are currently accepted.

Regular motion and ServoJ use the same atomic acquisition path. Either can start
only when the controller is connected, RobotMode is `ENABLE`, and the control
state is `IDLE`. Acquisition changes the state to `REGULAR_MOTION` or `SERVO_J`
and creates the single active lease.

### `ServoControlSession`

One session represents one connection epoch. It owns:

- the periodic steady-clock worker;
- the latest target buffer;
- target and response watchdog state;
- initial-feedback and command-step safety checks;
- the active lease interaction with `ControlStateManager`; and
- the safe-stop strategy.

Target update methods do not write to TCP. The worker copies the latest target,
revalidates it, checks the lease immediately before sending, and calls
`MotionCommander::servoJ()`.

### `ServoKinematicsValidator`

The validator checks finite input, individual MG400 joint limits, and coupled
constraints using the canonical kinematics implementation in `mg400_common`.
It returns structured safety codes and diagnostic detail without depending on
ROS.

### `ServoControlRosInterface`

The ROS interface creates:

- the `enable_servo_j` service;
- the `servo_j/target` subscription;
- the retained `control_state` publisher; and
- the retained `servo_error` publisher.

It converts ROS messages to the session's SI-unit target and converts internal
state snapshots to ROS messages. Periodic work, watchdogs, and safe stopping
remain in `ServoControlSession`.

### `MG400Node`

The lifecycle node owns the error latches, creates a fresh session for each
connection epoch, installs it in the ROS interface, and coordinates session
stop/destruction before TCP dependencies are destroyed.

## Concurrency model

`ControlStateManager`, `ServoControlSession`, feedback state, and error latches
protect their own state. The ROS interface serializes Servo-state transitions
and rejects target callbacks while a transition is in progress. TCP calls and
stop strategies run without holding the session state mutex.

The important ordering rule is:

1. close target admission;
2. stop and join periodic work;
3. execute and confirm the stop strategy;
4. release the lease; and
5. destroy the session before its TCP dependencies.

## Error publication

Safety and operational states are owned above individual connection sessions so
their first-fault snapshots survive session replacement. The ROS interface
registers RAII callbacks on both states and publishes a combined retained
snapshot whenever either state changes.

## Tests

| Test | Coverage |
| --- | --- |
| `test_control_state_manager.cpp` | State transitions, leases, RobotMode, and regular-motion exclusion |
| `test_servo_kinematics_validator.cpp` | Joint and coupled constraints |
| `test_servo_control_session.cpp` | Latest-only transmission, feedback checks, watchdog, responses, stop retry, and stale epochs |
| `test_servo_control_ros_interface.cpp` | ROS service contract, target admission, lifecycle behavior, QoS, and error publication |
| `test_motion_commander.cpp` | TCP command formatting |
