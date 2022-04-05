#!/usr/bin/env bash

# Clear error
ros2 service call /mg400/clear_error mg400_msgs/srv/ClearError

# Turn on
ros2 service call /mg400/enable_robot mg400_msgs/srv/EnableRobot

# Move
## Roll
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 340.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 0.0, y: -340.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_j mg400_msgs/srv/MovJ \
  "{x: 340.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"

ros2 service call /mg400/mov_l mg400_msgs/srv/MovL \
  "{x: 200.0, y: 50.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_l mg400_msgs/srv/MovL \
  "{x: 400.0, y: -50.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"
ros2 service call /mg400/mov_l mg400_msgs/srv/MovL \
  "{x: 340.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}"

sleep 1

# Turn off
ros2 service call /mg400/disable_robot mg400_msgs/srv/DisableRobot
