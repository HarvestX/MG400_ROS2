# Real-time feedback message for MG400 robot.
# Field names are the bit table names converted to ROS 2 snake_case.
# Numeric values are published as received from the real-time feedback packet.

builtin_interfaces/Time stamp

uint16 message_size
uint64 digital_inputs
uint64 digital_outputs
uint64 robot_mode
uint64 time_stamp
uint64 run_time
uint64 test_value
float64 speed_scaling
float64 v_robot
float64 i_robot
float64 program_state
uint8[2] safety_io_in
uint8[2] safety_io_out

float64[6] q_target
float64[6] qd_target
float64[6] qdd_target
float64[6] i_target
float64[6] m_target
float64[6] q_actual
float64[6] qd_actual
float64[6] i_actual
float64[6] actual_tcp_force
float64[6] tool_vector_actual
float64[6] tcp_speed_actual
float64[6] tcp_force
float64[6] tool_vector_target
float64[6] tcp_speed_target
float64[6] motor_temperatures
float64[6] joint_modes
float64[6] v_actual

uint8[4] hand_type
uint8 user
uint8 tool
uint8 run_queued_cmd
uint8 pause_cmd_flag
uint8 velocity_ratio
uint8 acceleration_ratio
uint8 xyz_velocity_ratio
uint8 r_velocity_ratio
uint8 xyz_acceleration_ratio
uint8 r_acceleration_ratio
uint8 brake_status
uint8 enable_status
uint8 drag_status
uint8 running_status
uint8 error_status
uint8 jog_status_cr
uint8 cr_robot_type
uint8 drag_button_signal
uint8 enable_button_signal
uint8 record_button_signal
uint8 reappear_button_signal
uint8 jaw_button_signal
uint8 six_force_online
uint8 collision_state
uint8 arm_approach_state
uint8 j4_approach_state
uint8 j5_approach_state
uint8 j6_approach_state

float64 vibration_dis_z
uint64 current_command_id
float64[6] m_actual
float64 load
float64 center_x
float64 center_y
float64 center_z
float64[6] user_coordinates
float64[6] tool_coordinates
float64[6] six_force_value
float64[4] target_quaternion
float64[4] actual_quaternion
uint16 auto_manual_mode
uint16 export_status
uint8 safety_state
uint8 safe_state
