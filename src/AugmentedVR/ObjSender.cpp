//
// Created by nsl on 9/23/16.
//

#include "ObjSender.hpp"


ObjSender::ObjSender(AugmentedVR *myAVR, string commPath) : myAVR(myAVR), commPath(commPath) {
}

ObjSender::~ObjSender() {
//    TcwFile.release();
//
//    PCFile.release();
}


void ObjSender::sendFrame(){
    writeFrame();
    writeTcw();
    writeTimeStamp();
    writePC();
    writeDynamicPC();
    writeObjectMotionVec();
    writeLowPassObjectMotionVec();
}

void ObjSender::writeFrame(){
    char tmpstr[100];
//    sprintf(tmpstr, "TcwFrame%d",myAVR->frameSeq);
    sprintf(tmpstr, "/cam%d/Frame%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    TcwFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    TcwFile << FRAME << myAVR->LastFrame.FrameLeft;
    TcwFile.release();
}

void ObjSender::writeTcw(){
    char tmpstr[100];
//    sprintf(tmpstr, "TcwFrame%d",myAVR->frameSeq);
    sprintf(tmpstr, "/cam%d/TcwFrame%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    TcwFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    TcwFile << TCW << myAVR->LastFrame.CamMotionMat;
    TcwFile.release();
}

void ObjSender::writeTimeStamp(){
//    char tmpstr[50];
//    sprintf(tmpstr, "timeFrame%d",myAVR->frameSeq);
//    TcwFile << tmpstr << (int)myAVR->frameTS; //TODO cannot support long??
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/timeFrame%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    TcwFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    TcwFile << TIMESTAMP << (int)myAVR->LastFrame.frameTS;
    TcwFile.release();
}


void ObjSender::writePC(){
//    char tmpstr[50];
//    sprintf(tmpstr, "fullPCFrame%d",myAVR->frameSeq);
//    PCFile << tmpstr << myAVR->pointcloud;
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/fullPCFrame%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    PCFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    PCFile << PC << myAVR->LastFrame.pointcloud;
    PCFile.release();
}

void ObjSender::writeDynamicPC(){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/dynamicPCFrame%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    //TODO investigate 0
    dynamicPCFile << DYNAMICPC << myAVR->lastStereoData[0].DynamicPC;
//    dynamicPCFile << DYNAMICPC << myAVR->LastFrame.DynamicPC;
    dynamicPCFile.release();
}



void ObjSender::writeObjectMotionVec(){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    cout << "sending motion vec\n";
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/objectMotionVec%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    motionFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    //TODO investigate 0
    motionFile << MOTIONVEC <<  myAVR->lastStereoData[0].ObjectMotionVec;
    cout << myAVR->lastStereoData[0].ObjectMotionVec << endl;
    motionFile.release();
}

void ObjSender::writeLowPassObjectMotionVec(){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    cout << "sending low pass motion vec\n";
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/LowPassObjectMotionVec%d.yml", myAVR->CamId, myAVR->LastFrame.frameSeq);
    motionFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    motionFile << MOTIONVEC <<  myAVR->lastStereoData[0].LowPass_ObjectMotionVec;
    cout << myAVR->lastStereoData[0].LowPass_ObjectMotionVec << endl;
    motionFile.release();
}

//void ObjSender::logFrame(){
//    char tmpstr[100];
//    sprintf(tmpstr, "/cam%d/FrameInfo.txt", myAVR->CamId);
//    logFile.open(commPath+tmpstr, fstream::app);
//    logFile << "Frame " << myAVR->LastFrame.frameSeq << ": TS: "<< myAVR->LastFrame.ZEDTS
//            << ": Obj MotionVec: " << myAVR->ObjectMotionVec << ": dist: " << norm(myAVR->ObjectMotionVec)
//            << ": LP_MotionVec: " << myAVR-> Log_LowPassMotionVec << ": dist: " << norm(myAVR->Log_LowPassMotionVec) << endl;
//    logFile.close();
//}