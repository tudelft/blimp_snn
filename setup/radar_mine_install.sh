#!/bin/bash

# Create the catkin workspace
FOLDER_NAME='ros_radar_mine'
BRANCH='master'
mkdir -p ~/Pi_Git/${FOLDER_NAME}/catkin_ws/src
cd ~/Pi_Git/${FOLDER_NAME}/catkin_ws/
#catkin_make

# Add the catkin_simple package
cd ~/Pi_Git/${FOLDER_NAME}/catkin_ws/src
mkdir catkin_simple
cd catkin_simple
git clone https://github.com/catkin/catkin_simple .
rm -rf .git

# Add the PlotJuggler package
cd ~/Pi_Git/${FOLDER_NAME}/catkin_ws/src
git clone https://github.com/PlotJuggler/plotjuggler_msgs.git
git clone https://github.com/facontidavide/PlotJuggler.git
git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
cd src/PlotJuggler/
rm -rf .git
cd ../plotjuggler_msgs/
rm -rf .git
cd ../plotjuggler-ros-plugins/
rm -rf .git
#catkin build

# - If any problems:
# 1) First download PlotJuggler with snap following the instructions at: https://github.com/facontidavide/PlotJuggler#just-download-and-run
# 2) Also, run: sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
# 3) More useful instructions can be found at: https://github.com/PlotJuggler/plotjuggler-ros-plugins
# 4) To use in a launch file, add:
# <node name="plotjuggler" pkg="plotjuggler" type="plotjuggler" args="--layout /home/marina/Pi_Git/ros_radar_mine/myLayout3.xml"></node>
# 5) For help, do: plotjuggler -h

# Include the radar_position2go repo
cd
mkdir github
cd github/
git clone https://github.com/marina-go-al/Pi_Git .
git fetch origin
git checkout ${BRANCH}
cp -r ~/github/ros_radar_mine/catkin_ws/src/radar_position2go ~/Pi_Git/${FOLDER_NAME}/catkin_ws/src

# Build the ROS drivers and server for the radar
cd ~/Pi_Git/${FOLDER_NAME}/catkin_ws/
catkin build

# Source the workspace
source ~/Pi_Git/${FOLDER_NAME}/catkin_ws/devel/setup.bash