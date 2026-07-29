# ServoJ mode

This document describes the ROS 2 ServoJ API provided by MG400_ROS2.

ServoJ accepts four joint-angle targets and sends the latest accepted target to
the MG400 controller at a fixed period. Targets, leases, feedback checks, and
safe stopping are handled independently from the regular motion APIs.

## Public ROS API

### Control state

`mg400_msgs/msg/ControlState` reports ROS-side motion ownership independently
from the controller's `RobotMode`:

| State | Value | Meaning |
| --- | ---: | --- |
| `UNAVAILABLE` | 0 | The controller is disconnected or cannot accept Servo control |
| `IDLE` | 1 | No ROS motion API owns motion |
| `SERVO_J` | 2 | A ServoJ lease owns motion |
| `REGULAR_MOTION` | 3 | MovJ, MovL, Jog, CommandQueue, or another regular API owns motion |

The current state is published on `control_state` with reliable,
transient-local, keep-last-one QoS.

### Starting and stopping

The `enable_servo_j` service uses `mg400_msgs/srv/EnableServoJ`.

To start ServoJ:

```text
enable = true
```

A successful response returns the single active, non-zero lease ID. The lease
must be included in every ServoJ target.

The response's `enabled` field reports actual ServoJ ownership.

To stop ServoJ:

```text
enable = false
```

The driver first closes target admission, performs the configured safe-stop
strategy, confirms the controller state, and only then releases the lease.
Failed stop confirmation retains the active lease internally so another
`enable = false` request can retry the stop. Disable requests require no ID.

### Target topic

ServoJ targets are published to `servo_j/target` as
`mg400_msgs/msg/ServoJ`:

```text
uint64 lease_id
float64[4] joint_angles
```

Joint angles use radians. The subscription keeps only the latest target.

Example:

```bash
ros2 topic pub --rate 33 /mg400/servo_j/target mg400_msgs/msg/ServoJ \
  "{lease_id: 1, joint_angles: [0.0, 0.2, 0.5, 0.0]}"
```

## Target processing

`ServoControlSession` performs the following checks before sending a target:

1. the target contains only finite values;
2. the lease matches the active ServoJ session;
3. the target satisfies MG400 individual and coupled-joint constraints;
4. the first command is close to fresh realtime joint feedback;
5. later commands are close to the last successfully transmitted command; and
6. the lease is still valid immediately before the TCP send.

The latest accepted target is converted from radians to degrees and sent as:

```text
ServoJ(J1,J2,J3,J4)
```

The periodic worker does not build a command queue. A newer target replaces the
previous buffered target.

## Watchdog and stopping

The target watchdog starts when ServoJ acquires a lease. Each accepted target
refreshes its steady-clock deadline. If the deadline expires, the session stops
target admission, executes the safe-stop strategy, and releases ownership only
after stop confirmation.

The same stop path is used for explicit stops, watchdog expiry, response
failures, safety violations, lifecycle deactivation, and destruction fallback.

## Safety and operational errors

`servo_error` publishes a retained `mg400_msgs/msg/ServoError` snapshot.

ServoJ safety codes are:

| Code | Meaning |
| --- | --- |
| `SAFETY_NONE` | No latched safety violation |
| `SAFETY_SERVO_J_JOINT_LIMIT` | An individual joint limit was violated |
| `SAFETY_SERVO_J_COUPLED_LIMIT` | A coupled-joint constraint was violated |
| `SAFETY_SERVO_J_COMMAND_DISCONTINUITY` | A target changed too far from its reference |
| `SAFETY_REALTIME_FEEDBACK_UNAVAILABLE` | Initial feedback was absent, stale, or from another connection epoch |

Operational errors cover watchdog expiry, TCP send and response failures,
connection loss, lease loss, stop failures, and internal errors. Safety and
operational states are first-fault latches. A successful later ServoJ start
rearms both latches.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `servo.send_period_ms` | 30 | Periodic TCP send interval |
| `servo.target_watchdog_ms` | 30000 | Maximum time without an accepted target |
| `servo.stop_timeout_ms` | 2000 | Safe-stop confirmation timeout |
| `servo.stop_confirmation_poll_ms` | 10 | RobotMode polling interval during stop |
| `servo.safety.feedback_timeout_ms` | 100 | Maximum age of initial realtime feedback |
| `servo.safety.max_initial_joint_distance_rad` | 0.0872665 | Maximum first-target distance from feedback |
| `servo.safety.max_joint_step_rad` | 0.0174533 | Maximum later step from the last successful command |

All durations and safety thresholds must be positive. Safety thresholds must
also be finite.

## Operation GUI

`mg400_operation_gui` provides ServoJ joint sliders, current-joint loading,
lease start/stop, and periodic target publication. It initializes targets from
`realtime_feedback` and applies a configurable low-pass filter before publishing.
