#!/bin/bash
# Assume a brand new ubuntu 16 system
# Install cuda first
sudo su
# packages
sudo apt-get install libopenexr-dev libeigen3-dev libboost-all-dev libgtk2.0-dev libqt4-dev libopenni-dev libopenni2-dev libvtk6-dev libxmu-dev libxi-dev libflann-dev libcpprest-dev cmake libprotobuf-dev libproj-dev
# ZED SDK
mkdir soft
cd soft
wget https://www.stereolabs.com/developers/downloads/ZED_SDK_Linux_Ubuntu16_v2.1.2.run
chmod +x ZED_SDK_Linux_Ubuntu16_v2.1.2.run
./ZED_SDK_Linux_Ubuntu16_v2.1.2.run
# PCL
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz
tar -xzvf pcl-pcl-1.7.2.tar.gz
cd pcl-pcl-1.7.2 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make -j8 install
# Pangolin
cd ../../
wget https://github.com/stevenlovegrove/Pangolin/archive/v0.5.tar.gz
tar -xzvf Pangolin-0.5.tar.gz
cd Pangolin-0.5
mkdir build
cd build
cmake ..
make -j8
make -j8 install
# OpenCV with Contrib
cd ../../
git clone https://github.com/opencv/opencv
git clone https://github.com/opencv/opencv_contrib
cd opencv
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j8
sudo make -j8 install



