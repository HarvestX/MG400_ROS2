#!/usr/bin/env bash

# Clear error
ros2 service call /mg400/clear_error mg400_msgs/srv/ClearError

# Turn on
ros2 service call /mg400/enable_robot mg400_msgs/srv/EnableRobot

# Move
## Roll
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.34, y: 0.0, z: 0.0, r: 0.0}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.0, y: -0.34, z: 0.0, r: 1.57}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.34, y: 0.0, z: 0.0, r: 0.0}"

sleep 5

# Turn off
ros2 service call /mg400/disable_robot mg400_msgs/srv/DisableRobot
