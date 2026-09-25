# ServoJ setpoint for the four MG400 joints, in radians.
# J3 is the controller's absolute angle, not the relative ROS j3_1 angle.
# Publish only while holding the session returned by servo_j_session.
uint64 session_id
float64[4] joint_positions
# Point execution time in seconds. Used when the configured wire format includes t.
float64 t
