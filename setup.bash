#!/usr/bin/env bash

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))
THIS_REPOSITORY_NAME="mg400"

# Install ROS2 dependency
## From git repos
colcon_ws=${THIS_PROJECT_ROOT}/../../
cd ${colcon_ws}/src
vcs import --recursive <${THIS_PROJECT_ROOT}/${THIS_REPOSITORY_NAME}.repos
unset colcon_ws

## From apt repositories
rosdep update
rosdep install -r -y -i \
  --from-paths ${THIS_PROJECT_ROOT} \
  --rosdistro $ROS_DISTRO \
  --skip-keys p9n_interface

unset THIS_FILE
unset THIS_PROJECT_ROOT

