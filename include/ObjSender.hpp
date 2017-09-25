//
// Created by nsl on 9/23/16.
//

#ifndef PROJECT_OBJSENDER_H
#define PROJECT_OBJSENDER_H


#include <opencv2/core/persistence.hpp>
#include "AugmentedVR.hpp"

class ObjSender {

private:
    AugmentedVR *myAVR;
    cv::FileStorage TcwFile;
    cv::FileStorage PCFile;
    cv::FileStorage dynamicPCFile;
    cv::FileStorage motionFile;
//    fstream TcwFile;
//    fstream PCFile;

    string commPath;
public:
    ObjSender(AugmentedVR *myAVR, string commPath);

    ~ObjSender();

    void writeFrame();
    void writeTcw();
    void writeTimeStamp();
    void writePC();
    void writeDynamicPC();
    void writeObjectMotionVec();
    void writeLowPassObjectMotionVec();
    void logFrame();

    void sendFrame();
};


#endif //PROJECT_OBJSENDER_H
