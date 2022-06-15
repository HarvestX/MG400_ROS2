#!/usr/bin/env bash

# Clear error
ros2 service call /mg400/clear_error mg400_msgs/srv/ClearError

# Turn on
ros2 service call /mg400/enable_robot mg400_msgs/srv/EnableRobot

#ros2 service call /mg400/get_error_id mg400_msgs/srv/GetErrorID
# Move
## Roll
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.34, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.0, y: -0.34, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.34, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"

ros2 service call /mg400/mov_l mg400_msgs/srv/MovL \
  "{x: 0.2, y: 0.05, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_l mg400_msgs/srv/MovL \
  "{x: 0.4, y: -0.05, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"

#ros2 service call /mg400/get_error_id mg400_msgs/srv/GetErrorID
ros2 service call /mg400/mov_l mg400_msgs/srv/MovL \
  "{x: 0.34, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \

sleep 1

# Turn off
ros2 service call /mg400/disable_robot mg400_msgs/srv/DisableRobot
