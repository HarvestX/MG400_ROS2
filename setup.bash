#!/usr/bin/env bash

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))

# Install python packages
python3 -m pip install -U -r $THIS_PROJECT_ROOT/requirements.txt

# Install ROS2 dependency
unset colcon_ws
rosdep update
rosdep install -r -y -i --from-paths ${THIS_PROJECT_ROOT} --rosdistro $ROS_DISTRO

unset THIS_FILE
unset THIS_PROJECT_ROOT
