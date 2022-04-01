# AVR: Augmented Vehicular Reality

AVR extends vehicular vision to see beyond obstruction by sharing views through V2V communications.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/9rOtH3hDcw8/0.jpg)](https://www.youtube.com/watch?v=9rOtH3hDcw8)


Note: 
- We are actively working and updating this repo to incorporate more features and datasets.
- For those who want to play with the code, We will release a mini-dataset and release a stable version soon.


## Getting started

### Prerequisites

- Ubuntu 16.04
- Latest [CMake](https://cmake.org/download/)
- [ZED SDK **2.0**](https://www.stereolabs.com/developers/)
- [CUDA **8.0**](https://developer.nvidia.com/cuda-downloads)
- sudo apt-get install libopenexr-dev libeigen3-dev libboost-all-dev libgtk2.0-dev libqt4-dev libopenni-dev libopenni2-dev libvtk6-dev libxmu-dev libxi-dev libflann-dev libcpprest-dev cmake libproj-dev
- [PCL **1.7.2**](https://github.com/PointCloudLibrary/pcl) installed with its visualization module
- [Pangolin](https://github.com/stevenlovegrove/Pangolin), get a release version 
- [ProtoBuf](https://github.com/google/protobuf/blob/master/src/README.md)
- [OpenCV **3.1**](http://opencv.org/downloads.html) with [OpenCV_Contrib](https://github.com/opencv/opencv_contrib), get a releaser version from both



## Build the program

    sh pre-build.sh  //build 3rd party libraries and untar vocabulary
    mkdir build
    cd build
    cmake ..
    make
    
## Citation
```bibtex
@inproceedings{avr,
    author = {Qiu, Hang and Ahmad, Fawad and Bai, Fan and Gruteser, Marco and Govindan, Ramesh},
    title = {AVR: Augmented Vehicular Reality},
    year = {2018},
    url = {https://doi.org/10.1145/3210240.3210319},
    doi = {10.1145/3210240.3210319},
    booktitle = {Proceedings of the 16th Annual International Conference on Mobile Systems, Applications, and Services},
    pages = {81–95},
    numpages = {15},
    keywords = {Autonomous Cars, Extended Vision, Collaborative Sensing},
    location = {Munich, Germany},
    series = {MobiSys '18}
}
```
