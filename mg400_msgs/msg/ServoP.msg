# Cartesian target for cyclic ServoP control.
# Position is expressed in meters. For the four-axis MG400, only the yaw
# component of the orientation is used.
uint64 lease_id  # Lease returned when SERVO_P control starts.
geometry_msgs/PoseStamped pose
