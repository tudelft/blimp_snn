#Prepare the working environment
sudo apt-get install build-essential cmake pkg-config
sudo apt -y install python-dev python-pip python3-dev python3-pip python3-testresources
sudo apt -y install python-numpy python3-numpy
#Install required libraries
sudo apt -y install libblas-dev libblas-test liblapack-dev libatlas-base-dev libopenblas-base libopenblas-dev
sudo apt -y install libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt -y install libxvidcore-dev libx264-dev
sudo apt -y install libgtk2.0-dev libgtk-3-dev libcanberra-gtk*
sudo apt -y install libtiff5-dev libeigen3-dev libtheora-dev
sudo apt -y install libvorbis-dev sphinx-common libtbb-dev yasm libopencore-amrwb-dev
sudo apt -y install libopenexr-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libavutil-dev libavfilter-dev
sudo apt -y install libavresample-dev ffmpeg libdc1394-22-dev libwebp-dev
sudo apt -y install libjpeg8-dev libxine2-dev libmp3lame-dev libopencore-amrnb-dev libprotobuf-dev
sudo apt -y install protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libhdf5-dev
sudo apt -y install qt5-default v4l-utils
sudo apt -y install libtbb2
#Download OpenCV and OpenCV_Contrib
wget -O opencv.zip https://github.com/Itseez/opencv/archive/4.1.2.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/4.1.2.zip
unzip opencv_contrib.zip
#Rename folders
mv opencv-4.1.2 opencv
mv opencv_contrib-4.1.2 opencv_contrib
#Build OpenCV
cd opencv
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D PYTHON_EXECUTABLE=/usr/bin/python3.5 \
-D PYTHON2_EXECUTABLE=/usr/bin/python2.7 \
-D PYTHON3_EXECUTABLE=/usr/bin/python3.5 \
-D BUILD_EXAMPLES=ON ..
#Before compilation and installation, create a swapfile:
sudo dd if=/dev/zero of=/swapfile bs=1024 count=1M
sudo chown root.root /swapfile
sudo chmod 0600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
#Compile
make -j4
#Set up the Python environment (Python 2.7 or 3):
cd ~
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
#Install OpenCV:
cd ~/opencv/build/
sudo make install
sudo ldconfig
sudo apt update




