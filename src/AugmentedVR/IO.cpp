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
    char tmp_str[50];
    sprintf(tmp_str, "/home/nsl/imgs/cam%d/depth.txt", myAVR->CamId);
    depthFile.open(tmp_str);

    sprintf(tmp_str, "/home/nsl/imgs/cam%d/times.txt", myAVR->CamId);
    TimeFile.open(tmp_str);

    sprintf(tmp_str, "./FrameInfo%d.txt", myAVR->CamId);
    logFile.open(tmp_str);


//    sprintf(tmp_str, "./imgs/cam%d/Tcw.txt", myAVR->CamId);
//    TcwFile.open(".yml", cv::FileStorage::WRITE);



}

IO::~IO() {
    depthFile.close();
    logFile.close();
    TimeFile.close();
}


void IO::writeCurrentStereoFrame(){
    cout <<"writing frames " << FRAME_ID << endl;
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    char tmp_str[50];
    sprintf(tmp_str, "/home/nsl/imgs/cam%d/image_0/%06d.png", myAVR->CamId, FRAME_ID);
    imwrite(tmp_str,currFrame.FrameLeft);
    sprintf(tmp_str, "/home/nsl/imgs/cam%d/image_1/%06d.png", myAVR->CamId, FRAME_ID);
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
            << ": LP_MotionVec: " << myAVR-> Log_LowPassMotionVec  //TODO check valid
            << ": dist: " << norm(myAVR->Log_LowPassMotionVec) << endl;

}