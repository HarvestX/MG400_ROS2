# ROS sampling time; frame_id identifies the robot origin frame.
std_msgs/Header header

# Raw motor currents in amperes, ordered by physical axes J1, J2, J3, J4.
# No bias correction, sign adjustment, filtering, or torque conversion is applied.
float64[4] actual

# Controller target currents in amperes, in the same axis order.
float64[4] target
