#!/bin/bash
set -e

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

source /opt/ros/humble/setup.bash
source /home/user/dev_ws/ros2/install/local_setup.bash
source /usr/share/gazebo/setup.bash

export GAZEBO_MODEL_PATH=/home/user/dev_ws/ros2/src/sim_cf2/models:$GAZEBO_MODEL_PATH

exec "$@"