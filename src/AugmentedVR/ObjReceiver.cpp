//
// Created by nsl on 9/23/16.
//

#include "ObjReceiver.hpp"
#include <opencv2/core/mat.hpp>
#include <cpprest/json.h>
#include "cpprest/http_client.h"
#include "stdafx.hpp"

using namespace std;
using namespace web;
using namespace http;
using namespace utility;
using namespace http::client;

ObjReceiver::ObjReceiver(AugmentedVR *myAVR, const int CamId, string commPath) : myAVR(myAVR), RxCamId(CamId), commPath(commPath) {
//    char tmp_str[50];
//    sprintf(tmp_str, "./comm/cam%d/PointCloud.yml",CamId);
//    PCFile.open(tmp_str, cv::FileStorage::READ);
//    sprintf(tmp_str, "./comm/cam%d/Tcw.yml", CamId);
//    TcwFile.open(tmp_str, cv::FileStorage::READ);
//    sprintf(tmp_str, "./comm/cam%d/dynamicPC.yml", CamId);
//    dynamicPCFile.open(tmp_str, cv::FileStorage::READ);
    utility::string_t port = U("34568");
    utility::string_t address = U("http://127.0.0.1:");
    address.append(port);
    uri_builder uri(address);
    // connect to the base address
    client = new http_client(uri.to_uri());
}

ObjReceiver::~ObjReceiver() {
//    TcwFile.release();
//    PCFile.release();
    delete client;
}

cv::Mat ObjReceiver::AskForPointCloud(int FrameId){
    http_response getresponse = CheckResponse(client->request(methods::GET, U("/20")).get());
    V2VBuffer.open(getresponse.extract_string().get(), cv::FileStorage::READ + cv::FileStorage::MEMORY);
    cv::Mat ret;
    V2VBuffer[FRAME] >> ret;
    cout << ret << endl;
    return ret;
}


http_response ObjReceiver::CheckResponse(const http_response &response)
{
    ucout << response.to_string() << endl;
    return response;
}


struct WholeFrameWithMetaData ObjReceiver::readWholeFrameWithFullMetaData(int frameSeq){
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/Frame%d.yml", RxCamId, frameSeq);
    WholeFrameWithMetaData ret;
    WholeFrame.open(commPath+tmpstr, cv::FileStorage::READ);
    WholeFrame[FRAME] >> ret.Frame;
    WholeFrame[TCW] >> ret.Tcw;
    WholeFrame[TIMESTAMP] >> ret.Timestamp;
    WholeFrame[PC] >> ret.PC;
    WholeFrame[DYNAMICPC] >> ret.DynamicPC;
    WholeFrame[MOTIONVEC] >> ret.MotionVec;
    WholeFrame[LOWPASSMOTIONVEC] >> ret.LowPassMotionVec;
    WholeFrame.release();
    return ret;
}

struct WholeFrameWithMetaData ObjReceiver::readWholeFrameFromSeparateFiles(int frameSeq){
    WholeFrameWithMetaData ret;
    ret.Frame = readFrame(frameSeq);
    ret.Tcw = readTcw(frameSeq);
    ret.Timestamp = readTimeStamp(frameSeq);
    ret.PC = readPC(frameSeq);
    ret.DynamicPC = readDynamicPC(frameSeq);
    ret.MotionVec = readObjectMotionVec(frameSeq);
    ret.LowPassMotionVec= readLowPassObjectMotionVec(frameSeq);
    return ret;
}

cv::Mat ObjReceiver::readFrame(int frameSeq){
//    char tmp_str[50];
//    sprintf(tmp_str, "TcwFrame%d", frameSeq);
//    cv::Mat ret;
//    TcwFile[tmp_str] >> ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/Frame%d.yml", RxCamId, frameSeq);
    cv::Mat ret;
    TcwFile.open(commPath+tmpstr, cv::FileStorage::READ);
    TcwFile[FRAME] >> ret;
    TcwFile.release();
    return ret;
}

cv::Mat ObjReceiver::readTcw(int frameSeq){
//    char tmp_str[50];
//    sprintf(tmp_str, "TcwFrame%d", frameSeq);
//    cv::Mat ret;
//    TcwFile[tmp_str] >> ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/TcwFrame%d.yml", RxCamId, frameSeq);
    cv::Mat ret;
    TcwFile.open(commPath+tmpstr, cv::FileStorage::READ);
    TcwFile[TCW] >> ret;
    TcwFile.release();
    return ret;
}



long ObjReceiver::readTimeStamp(int frameSeq){
//    char tmp_str[50];
//    sprintf(tmp_str, "timeFrame%d", frameSeq);
//    int ret;
//    TcwFile[tmp_str] >> ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/timeFrame%d.yml", RxCamId, frameSeq);
    int ret;
    TcwFile.open(commPath+tmpstr, cv::FileStorage::READ);
    TcwFile[TIMESTAMP] >> ret;
    TcwFile.release();
    return (long)ret;
}
cv::Mat ObjReceiver::readPC(int frameSeq){
//    char tmp_str[50];
//    sprintf(tmp_str, "PCFrame%d", frameSeq);
//    cv::Mat ret;
//    PCFile[tmp_str] >> ret;
//    return ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/fullPCFrame%d.yml", RxCamId, frameSeq);
    cv::Mat ret;
    PCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    PCFile[PC] >> ret;
    PCFile.release();
    return ret;
}
cv::Mat ObjReceiver::readDynamicPC(int frameSeq){
//    char tmp_str[50];
//    sprintf(tmp_str, "dynamicPCFrame%d", frameSeq);
//    cv::Mat ret;
//    dynamicPCFile[tmp_str] >> ret;
//    return ret;
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/dynamicPCFrame%d.yml", RxCamId, frameSeq);
    cv::Mat ret;
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    dynamicPCFile[DYNAMICPC] >> ret;
    dynamicPCFile.release();
    return ret;
}



cv::Mat ObjReceiver::readObjectMotionVec(int frameSeq){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    char tmpstr[100];
    cv::Mat ret;
    sprintf(tmpstr, "/cam%d/objectMotionVec%d.yml", RxCamId, frameSeq);
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    dynamicPCFile[MOTIONVEC]>> ret;
    dynamicPCFile.release();
    return ret;
}

cv::Mat ObjReceiver::readLowPassObjectMotionVec(int frameSeq){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    char tmpstr[100];
    cv::Mat ret;
    sprintf(tmpstr, "/cam%d/LowPassObjectMotionVec%d.yml", RxCamId, frameSeq);
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    dynamicPCFile[LOWPASSMOTIONVEC]>> ret;
    dynamicPCFile.release();
    return ret;
}