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
#include "mySocket.hpp"

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

    mySocket mSock;
    int sockfd;

public:

    ObjReceiver(AugmentedVR *myAVR, const int CamId, string commPath);
    ~ObjReceiver();

    void initMySocket();
    void initCPPREST();
    void ReceivePointCloudStream();

    struct WholeFrameWithMetaData readWholeFrameWithFullMetaData(int frameSeq);
    struct WholeFrameWithMetaData readWholeFrameFromSeparateFiles(int frameSeq);

    void readFrame(int frameSeq, cv::Mat & ret);
    void readTcw(int frameSeq, cv::Mat& ret);
    unsigned long long int readTimeStamp(int frameSeq);
    void readPC(int frameSeq, cv::Mat&ret);
    void readDynamicPC(int frameSeq, cv::Mat&ret);
    void readObjectMotionVec(int frameSeq, cv::Mat&ret);
    void readLowPassObjectMotionVec(int frameSeq,cv::Mat&ret);

    http_response CheckResponse(const http_response &response);
    bool AskForLatestPC_TCW_TIME_CPPREST(AugmentedVR *Node);

};


#endif //PROJECT_OBJRECEIVER_H
