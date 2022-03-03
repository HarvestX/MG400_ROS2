#!/usr/bin/env bash


THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=`realpath $(dirname $(realpath $THIS_FILE))`

# Install python packages
python3 -m pip install -U -r $THIS_PROJECT_ROOT/requirements.txt

# Install ROS2 dependency
colcon_ws=${THIS_PROJECT_ROOT}/../../
cd ${colcon_ws}/src
vcs import < h6x_eye/h6x_eye.repos
unset colcon_ws
rosdep update
rosdep install -r -y -i --from-paths ./ --rosdistro $ROS_DISTRO


unset THIS_FILE
unset THIS_PROJECT_ROOT
