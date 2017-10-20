//
// Created by nsl on 9/23/16.
//

#ifndef PROJECT_OBJRECEIVER_H
#define PROJECT_OBJRECEIVER_H


#include <opencv2/core/mat.hpp>
#include "AugmentedVR.hpp"
#include <cpprest/json.h>
#include "cpprest/http_client.h"
#include "stdafx.hpp"

using namespace std;
using namespace web;
using namespace http;
using namespace utility;
using namespace http::client;

struct WholeFrameWithMetaData{
    cv::Mat Frame;
    cv::Mat Tcw;
    int Timestamp;
    cv::Mat PC;
    cv::Mat DynamicPC;
    cv::Mat MotionVec;
    cv::Mat LowPassMotionVec;
};

class ObjReceiver {

    AugmentedVR* myAVR;
    int RxCamId;
    cv::FileStorage WholeFrame;
    cv::FileStorage TcwFile;
    cv::FileStorage PCFile;
    cv::FileStorage dynamicPCFile;
//    fstream TcwFile;
//    fstream PCFile;
//    fstream dynamicPCFile;
    string commPath;

    http_client* client;
    cv::FileStorage V2VBuffer;


public:

    ObjReceiver(AugmentedVR *myAVR, const int CamId, string commPath);
    ~ObjReceiver();

    struct WholeFrameWithMetaData readWholeFrameWithFullMetaData(int frameSeq);
    struct WholeFrameWithMetaData readWholeFrameFromSeparateFiles(int frameSeq);

    cv::Mat readFrame(int frameSeq);
    cv::Mat readTcw(int frameSeq);
    long readTimeStamp(int frameSeq);
    cv::Mat readPC(int frameSeq);
    cv::Mat readDynamicPC(int frameSeq);
    cv::Mat readObjectMotionVec(int frameSeq);
    cv::Mat readLowPassObjectMotionVec(int frameSeq);

    http_response CheckResponse(const http_response &response);
    bool AskForLatestPC_TCW_TIME(AugmentedVR *Node);

};


#endif //PROJECT_OBJRECEIVER_H
