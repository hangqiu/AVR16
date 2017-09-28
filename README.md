# AVR: Augmented Vehicular Reality

AVR extends vehicular vision to see beyond obstruction by sharing views through V2V communications.


## Getting started

### Prerequisites

- Ubuntu 16.04
- [ZED SDK **2.0**](https://www.stereolabs.com/developers/)
- [CUDA **8.0**](https://developer.nvidia.com/cuda-downloads)
- [PCL **1.7.2**](https://github.com/PointCloudLibrary/pcl) installed with its visualization module
- [Pangolin](https://github.com/stevenlovegrove/Pangolin), get a release version 
- [ProtoBuf](https://github.com/google/protobuf/blob/master/src/README.md)
- Latest [CMake](https://cmake.org/download/)
- sudo apt-get install libopenexr-dev libeigen3-dev libboost-all-dev libgtk2.0-dev libqt4-dev libopenni-dev libopenni2-dev libvtk5-dev libxmu-dev libxi-dev libflann-dev
- [OpenCV **3.1**](http://opencv.org/downloads.html) with [OpenCV_Contrib](https://github.com/opencv/opencv_contrib), get a releaser version from both



## Build the program

    sh pre-build.sh  //build 3rd party libraries and untar vocabulary
    mkdir build
    cd build
    cmake ..
    make
