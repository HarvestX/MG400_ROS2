# Joint angles in radian. MG400 exposes four controllable joints.
# The Dobot TCP/IP ServoJ command uses degree and requires J5/J6 zero-fill.
float64[4] joint_angles

# Unused compatibility fields. Keep them 0.0.
float64 t
float64 aheadtime
float64 gain
