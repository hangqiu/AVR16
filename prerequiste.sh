#!/bin/bash
SOFTDIR=/home/hang/software/
sudo apt-get install libopenexr-dev libeigen3-dev libboost-all-dev libgtk2.0-dev libqt4-dev libopenni-dev libopenni2-dev libvtk5-dev libxmu-dev libxi-dev libflann-dev
#install Pangolin
cd $SOFTDIR
wget https://github.com/stevenlovegrove/Pangolin/archive/v0.5.tar.gz
tar xvzf v0.5.tar.gz
cd Pangolin-0.5
mkdir build
cd build
cmake ..
sudo make -j8
sudo make -j8 install
#install PCL
cd $SOFTDIR
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz
tar xvzf pcl-1.7.2.tar.gz
cd pcl-pcl-1.7.2
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make -j8
sudo make -j8 install
