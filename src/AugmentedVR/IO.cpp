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
    TXRXFile.open("TXRX.txt");
}

IO::~IO() {
    depthFile.close();
    logFile.close();
    TimeFile.close();
    TXRXFile.close();
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

void IO::logTXRX(char* output){
    TXRXFile << output;
}

void IO::logTXRX(){
    RxFrame* rx = myAVR->RxBuffer.getCurrentRxFrame();
    TXRXFile << "Current FrameID, " << myAVR->TotalFrameSeq-2
//         << ", "<< myAVR->getCurrentAVRFrame_TimeStamp_FrameTS() / 1000
         <<","<< myAVR->getCurrentAVRFrame_AbsoluteTimeStamp() / 1000000 ;
    TXRXFile << ", " << getCurrentComputerTimeStamp_usec() / 1000;
    TXRXFile << ", Received Frame, " << rx->RxSeq
//         << ", " << rx->RxTimeStamp /1000
         << ", " << rx->RxTimeStamp_ZEDTS /1000000
         << endl;
    if (rx->RxMotionVecSeq.size()!=0){
        for (int i=0;i<rx->RxMotionVecSeq.size();i++){
            TXRXFile << "Current FrameID, " << myAVR->TotalFrameSeq-2
                 <<","<< myAVR->getCurrentAVRFrame_AbsoluteTimeStamp() / 1000000 ;
            TXRXFile << ", " << getCurrentComputerTimeStamp_usec() / 1000;
            TXRXFile << ", Received MV, " << rx->RxMotionVecSeq[i]
                 << ", " << rx->RxMotionVec_ZEDTS[i] /1000000
                 << endl;
        }
    }
}