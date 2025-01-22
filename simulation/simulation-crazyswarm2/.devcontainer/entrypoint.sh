#!/bin/bash
set -e

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

source /opt/ros/jazzy/setup.bash
source /home/user/dev_ws/ros2/src/install/local_setup.bash
export GZ_SIM_RESOURCE_PATH="/home/user/dev_ws/simulation/crazyflie-simulation/simulator_files/gazebo/"
export GAZEBO_MODEL_PATH=/home/user/dev_ws/simulation/crazyflie-simulation/simulator_files/gazebo/:$GAZEBO_MODEL_PATH

exec "$@"