# Current ROS-driver-local Servo error state.
#
# This is independent of MG400 controller ErrorID.
# The message is a retained state snapshot, not a one-shot event.
# Code NONE indicates that the corresponding latch is clear.

# Local ROS publish time. frame_id is unused.
std_msgs/Header header

# Servo safety violation codes.
# These constants are the canonical wire values used by
# mg400_interface::ServoSafetyViolationCode.
uint8 SAFETY_NONE=0
uint8 SAFETY_SERVO_J_JOINT_LIMIT=1
uint8 SAFETY_SERVO_J_COUPLED_LIMIT=2
uint8 SAFETY_SERVO_J_COMMAND_DISCONTINUITY=3
uint8 SAFETY_REALTIME_FEEDBACK_UNAVAILABLE=4

uint8 safety_violation_code
string safety_violation_message

# ROS-driver-local Servo operational error codes.
# These constants are the canonical wire values used by
# mg400_interface::ServoOperationalErrorCode.
uint8 OPERATIONAL_NONE=0
uint8 OPERATIONAL_WATCHDOG_TIMEOUT=1
uint8 OPERATIONAL_MOTION_TCP_SEND_FAILED=2
uint8 OPERATIONAL_SERVO_LEASE_LOST=3
uint8 OPERATIONAL_REALTIME_CONNECTION_LOST=4
uint8 OPERATIONAL_STOP_RESET_FAILED=5
uint8 OPERATIONAL_STOP_CONFIRMATION_TIMEOUT=6
uint8 OPERATIONAL_INTERNAL_ERROR=7

uint8 operational_error_code
string operational_error_message
