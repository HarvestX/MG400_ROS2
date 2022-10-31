#!/usr/bin/env bash

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))
THIS_REPOSITORY_NAME="mg400"

# Install ROS2 dependency
## From git repos
colcon_ws=$(realpath ${THIS_PROJECT_ROOT}/../../)

vcs import \
  --recursive \
  --input ${THIS_PROJECT_ROOT}/${THIS_REPOSITORY_NAME}.repos \
  ${colcon_ws}/src
vcs pull ${colcon_ws}/src

## From apt repositories
rosdep update
rosdep install -r -y -i \
  --from-paths ${colcon_ws}/src \
  --rosdistro $ROS_DISTRO

unset colcon_ws
unset THIS_FILE
unset THIS_PROJECT_ROOT
