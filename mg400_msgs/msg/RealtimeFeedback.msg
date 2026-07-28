# Decoded contents of one 1440-byte packet from the MG400 real-time feedback
# port (30004), based on the Dobot four-axis protocol revision 2024-04-19.
# Transport-validation fields (MessageSize and TestValue) and anonymous N/A
# byte ranges are omitted. Other named fields are retained even when their
# description says that they are reserved on MG400.
#
# Values use Dobot controller units, not ROS SI units. In particular, Cartesian
# positions are in millimetres and angular values are in degrees.

# Local time at which the complete packet was received. frame_id is optional.
std_msgs/Header header

uint8 ROBOT_TYPE_MG400=1
uint8 ROBOT_TYPE_M1_PRO=2
uint8 ROBOT_TYPE_M1_PRO_RS485=4

# Packet metadata and controller state (byte offsets 0-71).
uint64 digital_inputs
uint64 digital_outputs
uint64 robot_mode
uint64 controller_timestamp  # Unix timestamp in milliseconds.
float64 speed_scaling

# Controller electrical state (byte offsets 80-103).
float64 v_main   # Control-board voltage.
float64 v_robot  # Robot voltage.
float64 i_robot  # Robot current.

# Joint and TCP state (byte offsets 192-1007).
# The wire format always reserves six entries. Only the first four joint entries
# are meaningful for MG400.
float64[6] q_target
float64[6] qd_target
float64[6] qdd_target
float64[6] i_target
float64[6] m_target
float64[6] q_actual
float64[6] qd_actual
float64[6] i_actual
float64[6] actual_tcp_force  # Reserved by the MG400 four-axis protocol.
float64[6] tool_vector_actual  # [x, y, z, rx, ry, rz].
float64[6] tcp_speed_actual
float64[6] tcp_force
float64[6] tool_vector_target  # [x, y, z, rx, ry, rz].
float64[6] tcp_speed_target
float64[6] motor_temperatures
float64[6] joint_modes  # 8: position mode, 10: torque mode.
float64[6] v_actual

# Active coordinate systems, motion ratios, and status flags
# (byte offsets 1008-1037).
int8[4] hand_type
uint8 user_index
uint8 tool_index
uint8 run_queued_cmd
uint8 pause_cmd_flag
uint8 velocity_ratio
uint8 acceleration_ratio
uint8 jerk_ratio
uint8 xyz_velocity_ratio
uint8 r_velocity_ratio
uint8 xyz_acceleration_ratio
uint8 r_acceleration_ratio
uint8 xyz_jerk_ratio
uint8 r_jerk_ratio
uint8 brake_status  # Per-joint bit mask.
uint8 enable_status
uint8 drag_status
uint8 running_status
uint8 error_status
uint8 jog_status
uint8 robot_type
uint8 drag_button_signal      # Reserved by the MG400 four-axis protocol.
uint8 enable_button_signal    # Reserved by the MG400 four-axis protocol.
uint8 record_button_signal    # Reserved by the MG400 four-axis protocol.
uint8 reappear_button_signal  # Reserved by the MG400 four-axis protocol.
uint8 jaw_button_signal       # Reserved by the MG400 four-axis protocol.
uint8 six_force_online        # Reserved by the MG400 four-axis protocol.

# Torque, payload, and coordinate data (byte offsets 1120-1351).
float64[6] m_actual
float64 load  # Payload in kilograms.
float64 center_x  # Payload eccentric distance in millimetres.
float64 center_y
float64 center_z
float64[6] user  # User coordinates [x, y, z, rx, ry, rz].
float64[6] tool  # Tool coordinates [x, y, z, rx, ry, rz].
float64 trace_index
float64[6] six_force_value  # Reserved by the MG400 four-axis protocol.

# Quaternion order follows the wire protocol, not geometry_msgs/Quaternion.
float64[4] target_quaternion  # [qw, qx, qy, qz].
float64[4] actual_quaternion  # [qw, qx, qy, qz].
