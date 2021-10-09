# Create the catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
# Add the catkin_simple package
cd ~/catkin_ws/src
mkdir catkin_simple
cd catkin_simple
git clone https://github.com/catkin/catkin_simple .
rm -rf .git
# Include the radar_position2go repo
cd
mkdir github
cd github/
git clone https://github.com/JuSquare/Comp4Drones .
cp -r ~/github/odroid_ros_radar/radar_position2go/ ~/catkin_ws/src/
# Build the ROS drivers and server for the radar
cd ~/catkin_ws/
catkin_make radar_ros_driver
catkin_make radar_server
# Source the workspace
source ~/catkin_ws/devel/setup.bash