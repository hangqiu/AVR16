//
// Created by nsl on 9/20/16.
//


#include "IO.hpp"
#include "globals.hpp"

//IO::IO() {}

//void IO::init(){
//
//}

IO::IO(AugmentedVR *myAVR) : myAVR(myAVR) {
    depthFile.open(commPath + "/depth.txt");
    TimeFile.open(commPath + "/times.txt");
    logFile.open(commPath + "/FrameInfo.txt");
}

IO::~IO() {
    depthFile.close();
    logFile.close();
    TimeFile.close();
}


void IO::writeCurrentStereoFrame(){
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    cout <<"writing frames " << currFrame.frameSeq << endl;
    char tmp_str[50];
    sprintf(tmp_str, "/home/nsl/imgs/cam%d/image_0/%06d.png", myAVR->CamId, currFrame.frameSeq);
    imwrite(tmp_str,currFrame.FrameLeft);
    sprintf(tmp_str, "/home/nsl/imgs/cam%d/image_1/%06d.png", myAVR->CamId, currFrame.frameSeq);
    imwrite(tmp_str,currFrame.FrameRight);
    TimeFile << currFrame.frameTS << endl;
}


void IO::logCurrentFrame(){
//    char tmpstr[100];
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    logFile << "Frame " << currFrame.frameSeq
            << ": TS: " << currFrame.ZEDTS
            << ": Obj MotionVec: " << currFrame.ObjectMotionVec
            << ": dist: " << norm(currFrame.ObjectMotionVec)
            << ": LP_MotionVec: " << currFrame.LowPass_ObjectMotionVec
            << ": dist: " << norm(currFrame.LowPass_ObjectMotionVec)
            << ": Filtered Obj MotionVec: " << currFrame.FilteredObjectMotionVec
            << ": dist: " << norm(currFrame.FilteredObjectMotionVec)
            << ": Filtered LP_MotionVec: " << currFrame.LowPass_FilteredObjectMotionVec
            << ": dist: " << norm(currFrame.LowPass_FilteredObjectMotionVec)
            << endl;

}