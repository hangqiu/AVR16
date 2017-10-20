//
// Created by nsl on 9/23/16.
//

#ifndef PROJECT_OBJSENDER_H
#define PROJECT_OBJSENDER_H


#include <opencv2/core/persistence.hpp>
#include "AugmentedVR.hpp"
#include "ObjSenderHandler.hpp"

/*<< Defines the server. >>*/
//struct hello_world;
//typedef http::server<hello_world> server;


class ObjSender {

private:
    AugmentedVR *myAVR;
    cv::FileStorage WholeFrame;
    cv::FileStorage TcwFile;
    cv::FileStorage PCFile;
    cv::FileStorage dynamicPCFile;
    cv::FileStorage motionFile;
//    fstream TcwFile;
//    fstream PCFile;

    string commPath;

    std::unique_ptr<ObjSenderHandler> g_httpHandler;
public:



    ObjSender(AugmentedVR *myAVR, string commPath);

    ~ObjSender();

    void httpServerRun(string address, string port);

    void writeCurrentFrame();
    void writeCurrentTcw();
    void writeCurrentTimeStamp();
    void writePC();
    void writeDynamicPC();
    void writeObjectMotionVec();
    void writeLowPassObjectMotionVec();
    void logFrame();

    char* writeWholeFrameWithFullMetaData();
    char* writeFullFrame_PC_TCW_Time();

    void writeFrameInSeparateFile();


};


#endif //PROJECT_OBJSENDER_H
