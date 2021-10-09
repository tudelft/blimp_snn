#!/bin/bash

export ROS_IP=192.168.0.105
export ROS_MASTER_URI=http://192.168.0.105:11311
#export ROSLAUNCH_SSH_UNKNOWN=1

source /home/marina/Pi_Git/ros_radar_mine/catkin_ws/devel/setup.bash

exec "$@"
