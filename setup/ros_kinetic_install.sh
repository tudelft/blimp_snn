# Setup ROS Repositories
cd ~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Make sure your Debian package index is up-to-date
sudo apt-get update
sudo apt-get upgrade
# Install Bootstrap Dependencies
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
# Initializing rosdep
sudo rosdep init
rosdep update
# INSTALLATION:
# Create a catkin workspace
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws
# Install ROS-Comm: ROS package, build and communnication libraries. No GUI tools
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall
# If wstool init fails or is interrupted, you can resume the download by running:
# wstool update -j4 -t src
# RESOLVE DEPENDENCIES:
# Unavailable dependencies
mkdir -p ~/ros_catkin_ws/external_src
cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install
# Resolving dependencies with rosdep
cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:buster
# BUILDING THE CATKIN WORKSPACE
# Fixing liboost library issues
sudo apt remove libboost1.67-dev
sudo apt autoremove
sudo apt install -y libboost1.58-dev libboost1.58-all-dev
sudo apt install -y g++-5 gcc-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20
sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
sudo update-alternatives --set cc /usr/bin/gcc
sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++
sudo rm -rf ~/ros_catkin_ws/build_isolated
sudo rm -rf ~/ros_catkin_ws/devel_isolated
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
# Build
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
# Source the new ROS installation
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
