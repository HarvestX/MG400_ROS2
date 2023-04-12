#!/usr/bin/env bash

# Clear error
ros2 service call /mg400/clear_error mg400_msgs/srv/ClearError

# Turn on
ros2 service call /mg400/enable_robot mg400_msgs/srv/EnableRobot

# Set speed
ros2 service call /mg400/speed_factor mg400_msgs/srv/SpeedFactor "{ratio: 40}"

# Move
## Roll
ros2 action send_goal /mg400/mov_j mg400_msgs/action/MovJ \
  "{pose: {header: {frame_id: mg400_origin_link, stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 0.34, y: 0, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
ros2 action send_goal /mg400/mov_l mg400_msgs/action/MovL \
  "{pose: {header: {frame_id: mg400_end_effector_flange, stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: -0.1, y: 0.0, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.383, w: 0.924}}}}"
ros2 action send_goal /mg400/mov_l mg400_msgs/action/MovL \
  "{pose: {header: {frame_id: mg400_origin_link, stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 0.0, y: -0.34, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}"
ros2 action send_goal /mg400/mov_j mg400_msgs/action/MovJ \
  "{pose: {header: {frame_id: mg400_end_effector_flange, stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 0.0, y: 0.1, z: 0}, orientation: {x: 0.0, y: 0.0, z: -0.50, w: 0.866}}}}"

# Turn off
ros2 service call /mg400/disable_robot mg400_msgs/srv/DisableRobot
