//
// Created by nsl on 9/23/16.
//

#include "ObjReceiver.hpp"
#include <opencv2/core/mat.hpp>
#include <cpprest/json.h>
#include <include/UtilsPC.hpp>
#include "cpprest/http_client.h"
#include "stdafx.hpp"
#include "mySocket.hpp"

using namespace std;
using namespace web;
using namespace http;
using namespace utility;
using namespace http::client;
using namespace concurrency;
using namespace concurrency::streams;

ObjReceiver::ObjReceiver(AugmentedVR *myAVR, const int CamId, string commPath) : myAVR(myAVR), RxCamId(CamId), commPath(commPath) {


    if (!OfflineTXRX){
        initMySocket();
//    initCPPREST();
    }
}

ObjReceiver::~ObjReceiver() {
//    TcwFile.release();
//    PCFile.release();
    if (Parallel_TXRX){
        end = true;
        if (RX) delete rxstream;
    }
    delete client;
    delete &mSock;
}

void ObjReceiver::initMySocket(){
    mSock.Connect(ServerAddress.c_str(), ServerPort.c_str());
    if (Parallel_TXRX){
        if (RX){
            rxstream = new std::thread(&ObjReceiver::ReceiveLoop, this);
        }
    }
}


int ObjReceiver::ReceivePointCloudStream_FrameSeq(){
    char seqsizebuf[bufSize+1];
    mSock.Receive(seqsizebuf,bufSize);
    int seqbufsize = stoi(seqsizebuf);
    if (V2VDEBUG) cout << "seqbufsize:" << seqbufsize << endl;
    char seqbuf[seqbufsize+1];
    mSock.Receive(seqbuf,seqbufsize);
    int seq = stoi(seqbuf);
    if (V2VDEBUG)cout <<  "seq:" << seq << endl;
//    myAVR->RxSeq = seq;
    myAVR->RxBuffer.put_Seq(seq);
    return seq;
}

int ObjReceiver::ReceivePointCloudStream_TimeStamp_FrameTS(){
    char tssizebuf[bufSize+1];
    mSock.Receive(tssizebuf,bufSize);
    int tsbufsize = stoi(tssizebuf);
    if (V2VDEBUG)cout << "tsbufsize:" << tsbufsize<< endl;
    char tsbuf[tsbufsize+1];
    mSock.Receive(tsbuf,tsbufsize);
    int ts = stoi(tsbuf);
    if (V2VDEBUG)cout << "ts:" << ts << endl;
    myAVR->RxBuffer.put_TimeStamp(ts);
    return ts;
}

unsigned long long ObjReceiver::ReceivePointCloudStream_TimeStamp_ZEDTS(){
    char tssizebuf[bufSize+1];
    mSock.Receive(tssizebuf,bufSize);
    int tsbufsize = stoi(tssizebuf);
    if (V2VDEBUG)cout << "tsbufsize:" << tsbufsize<< endl;
    char tsbuf[tsbufsize+1];
    mSock.Receive(tsbuf,tsbufsize);
    unsigned long long ts = stoull(tsbuf);
    if (V2VDEBUG)cout << "ts:" << ts << endl;
    myAVR->RxBuffer.put_TimeStamp_ZEDTS(ts);
    return ts;
}


void ObjReceiver::ReceivePointCloudStream_TCW(){
    char tcwsizebuf[bufSize+1];
    mSock.Receive(tcwsizebuf,bufSize);
    int tcwbufsize = stoi(tcwsizebuf);
    if (V2VDEBUG)cout << "tcwbufsize:" << tcwbufsize<< endl;
    char buf[tcwbufsize+1];
    mSock.Receive(buf,tcwbufsize);
    cv::Mat tcw = cv::Mat(4,4,CV_32FC1, (void*)buf);
    if (V2VDEBUG)cout << "tcw\n"  << tcw << endl;
//    tcw.copyTo(myAVR->RxTCW);
    myAVR->RxBuffer.put_TCW(tcw);
}
void ObjReceiver::ReceivePointCloudStream_PC(){
#ifdef EVAL
    timeval tTXEnd, tTXStart;
    gettimeofday(&tTXStart, NULL);
    cout << "TimeStamp Start: " << tTXStart.tv_sec << "sec" << tTXStart.tv_usec << "usec" << endl;
#endif
    char pcsizebuf[bufSize+1];
    mSock.Receive(pcsizebuf,bufSize);
    int pcbufsize = stol(pcsizebuf);
    if (V2VDEBUG)cout << "pcbufsize:" << pcbufsize<< endl;
    char* pcbuf = (char*)malloc(pcbufsize+1);
//    char pcbuf[pcbufsize+1];
    mSock.ReceiveAll(pcbuf,pcbufsize);
    cv::Mat pc = cv::Mat(myAVR->height,myAVR->width, CV_32FC4, pcbuf);
    if (!pc.empty()){
//        myAVR->RxPC = cv::Mat();
//        pc.copyTo(myAVR->RxPC);
        myAVR->RxBuffer.put_PC(pc);
    }
//    debugPC(pc);
    free(pcbuf);
#ifdef EVAL
    gettimeofday(&tTXEnd, NULL);
    cout << "TimeStamp End: " << tTXEnd.tv_sec << "sec" << tTXEnd.tv_usec << "usec" << endl;
    cout << "PC RX: " <<double(tTXEnd.tv_sec-tTXStart.tv_sec)*1000 + double(tTXEnd.tv_usec-tTXStart.tv_usec) / 1000<< "ms"<< endl;
#endif
}
void ObjReceiver::ReceivePointCloudStream_Frame(){
    char imgsizebuf[bufSize+1];
    mSock.Receive(imgsizebuf,bufSize);
    int imgbufsize = stol(imgsizebuf);
    if (V2VDEBUG)cout << "imgsizebuf:"<< imgsizebuf<< endl;
    char* imgbuf = (char*)malloc(imgbufsize+1);
//    char pcbuf[pcbufsize+1];
    mSock.ReceiveAll(imgbuf,imgbufsize);
    cv::Mat img = cv::Mat(myAVR->height,myAVR->width, CV_8UC4, imgbuf);
    if (!img.empty()){
//        myAVR->RxFrame = cv::Mat();
//        img.copyTo(myAVR->RxFrame);
        myAVR->RxBuffer.put_FrameLeft(img);
    }
    if (V2VDEBUG)cv::imshow("received frame", img);
    free(imgbuf);
}
void ObjReceiver::ReceivePointCloudStream_ObjectMotionVec(){
    char mvsizebuf[bufSize+1];
    mSock.Receive(mvsizebuf,bufSize);
    int mvbufsize = stoi(mvsizebuf);
    if (V2VDEBUG)cout << "mvbufsize:" << mvbufsize<< endl;
    char buf[mvbufsize+1];
    mSock.Receive(buf,mvbufsize);
    cv::Mat mv = cv::Mat(1,1,CV_32FC3, (void*)buf);
    mv.copyTo(myAVR->RxMotionVec);
    myAVR->RxBuffer.put_MotionVec(mv);
    if (V2VDEBUG)cout << "mv\n"  << mv << endl;
}

void ObjReceiver::ReceivePointCloudStream(){
    int seq = ReceivePointCloudStream_FrameSeq();
    int ts = ReceivePointCloudStream_TimeStamp_FrameTS();
    unsigned long long tsZED = ReceivePointCloudStream_TimeStamp_ZEDTS();
    ReceivePointCloudStream_TCW();
    ReceivePointCloudStream_PC();
    ReceivePointCloudStream_Frame();
    ReceivePointCloudStream_ObjectMotionVec();
//    cout << "Current FrameID, " << myAVR->TotalFrameSeq-2
//         << ", "<< myAVR->getCurrentAVRFrame_TimeStamp_FrameTS() / 1000
//         <<","<< myAVR->getCurrentAVRFrame_AbsoluteTimeStamp() / 1000000 ;
//    cout << ", Received Frame, " << seq
//         << ", " << ts /1000
//         << ", " << tsZED /1000000
//         << endl;
    myAVR->RxBuffer.finishReceivingFrame();
}

void ObjReceiver::ReceiveLoop(){
    while(!end){
        ReceivePointCloudStream();
    }
}

void ObjReceiver::initCPPREST(){

    utility::string_t port = U(ServerPort.c_str());
    utility::string_t address = U(ServerAddress.c_str());
    address.append(port);
    uri_builder uri(address);
    // connect to the base address
    client = new http_client(uri.to_uri());
}

http_response ObjReceiver::CheckResponse(const http_response &response)
{
//    ucout << response.to_string() << endl;
//    ucout << "output to file\n";
//    fstream file("/home/hang/research/AVR16/tmp.yml");
//    file << response.to_string() << endl;
    return response;
}


//bool ObjReceiver::AskForLatestPC_TCW_TIME_CPPREST(AugmentedVR *Node){
//    char dir[100];
//    sprintf(dir, "/%d", 1000000);
//
//
//    return client->request(methods::GET, U(dir))
//          .then([](http_response response){
//              ucout << "waiting for payload..."<< endl;
//              return response.content_ready();
//          })
//          .then([this, Node](http_response getresponse){
//              if (getresponse.status_code()==status_codes::OK){
//                  try{
////                concurrency::streams::ostream stream;
////                concurrency::streams::container_buffer<std::string> inStringBuffer;
////                getresponse.body().read_to_end(inStringBuffer);
////                const string &res = inStringBuffer.collection();
////                V2VBuffer.open(res.c_str(), cv::FileStorage::READ + cv::FileStorage::MEMORY);
//                      V2VBuffer.open(getresponse.extract_string().get(), cv::FileStorage::READ + cv::FileStorage::MEMORY);
//                  }catch(...) { //slutils::cv::Exception
////                      cout << e.what();
//                      return false;
//                  }
//
//                  //        V2VBuffer[FRAME] >> Node->RxFrame;
//                  /// atomic reception
//                  cv::Mat RxPC, RxTCW;
//
//                  V2VBuffer[TIMESTAMP] >> Node->RxTimeStamp;
//                  V2VBuffer[PC] >> RxPC;
//                  V2VBuffer[TCW] >> RxTCW;
//
//                  if (RxPC.empty() || RxTCW.empty()) return false;
//
//                  RxPC.copyTo(Node->RxPC);
//                  RxTCW.copyTo(Node->RxTCW);
//                  return true;
//              }else{
//                  return false;
//              }
//          }).wait();
////          .then([=](http_response response){
////              return response.extract_string();
////
////          })
//}



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
    readFrame(frameSeq,ret.Frame);
    readTcw(frameSeq,ret.Tcw);
    ret.Timestamp = readTimeStamp(frameSeq);
    readPC(frameSeq,ret.PC);
    readDynamicPC(frameSeq,ret.DynamicPC);
    readObjectMotionVec(frameSeq,ret.MotionVec);
    readLowPassObjectMotionVec(frameSeq,ret.LowPassMotionVec);
    return ret;
}

void ObjReceiver::readFrame(int frameSeq, cv::Mat& ret){
//    char tmp_str[50];
//    sprintf(tmp_str, "TcwFrame%d", frameSeq);
//    cv::Mat ret;
//    TcwFile[tmp_str] >> ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/Frame%d.yml", RxCamId, frameSeq);
//    cv::Mat ret;
    TcwFile.open(commPath+tmpstr, cv::FileStorage::READ);
    TcwFile[FRAME] >> ret;
    TcwFile.release();
//    return ret;
}

void ObjReceiver::readTcw(int frameSeq, cv::Mat& ret){
//    char tmp_str[50];
//    sprintf(tmp_str, "TcwFrame%d", frameSeq);
//    cv::Mat ret;
//    TcwFile[tmp_str] >> ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/TcwFrame%d.yml", RxCamId, frameSeq);
//    cv::Mat ret;
    TcwFile.open(commPath+tmpstr, cv::FileStorage::READ);
    TcwFile[TCW] >> ret;
    TcwFile.release();
//    return ret;
}



unsigned long long int ObjReceiver::readTimeStamp(int frameSeq){
//    char tmp_str[50];
//    sprintf(tmp_str, "timeFrame%d", frameSeq);
//    int ret;
//    TcwFile[tmp_str] >> ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/timeFrame%d.yml", RxCamId, frameSeq);
    double ret;
    TcwFile.open(commPath+tmpstr, cv::FileStorage::READ);
    TcwFile[TIMESTAMP] >> ret;
    TcwFile.release();
    return (unsigned long long int)ret;
}
void ObjReceiver::readPC(int frameSeq, cv::Mat&ret){
//    char tmp_str[50];
//    sprintf(tmp_str, "PCFrame%d", frameSeq);
//    cv::Mat ret;
//    PCFile[tmp_str] >> ret;
//    return ret;

    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/fullPCFrame%d.yml", RxCamId, frameSeq);
//    cv::Mat ret;
    PCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    PCFile[PC] >> ret;
    PCFile.release();
//    return ret;
}
void ObjReceiver::readDynamicPC(int frameSeq,cv::Mat&ret){
//    char tmp_str[50];
//    sprintf(tmp_str, "dynamicPCFrame%d", frameSeq);
//    cv::Mat ret;
//    dynamicPCFile[tmp_str] >> ret;
//    return ret;
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/dynamicPCFrame%d.yml", RxCamId, frameSeq);
//    cv::Mat ret;
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    dynamicPCFile[DYNAMICPC] >> ret;
    dynamicPCFile.release();
//    return ret;
}



void ObjReceiver::readObjectMotionVec(int frameSeq, cv::Mat&ret){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    char tmpstr[100];
//    cv::Mat ret;
    sprintf(tmpstr, "/cam%d/objectMotionVec%d.yml", RxCamId, frameSeq);
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    dynamicPCFile[MOTIONVEC]>> ret;
    dynamicPCFile.release();
//    return ret;
}

void ObjReceiver::readLowPassObjectMotionVec(int frameSeq, cv::Mat&ret){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    char tmpstr[100];
//    cv::Mat ret;
    sprintf(tmpstr, "/cam%d/LowPassObjectMotionVec%d.yml", RxCamId, frameSeq);
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::READ);
    dynamicPCFile[LOWPASSMOTIONVEC]>> ret;
    dynamicPCFile.release();
//    return ret;
}


void ObjReceiver::ReadFromDisk(int frameSeqRx){
    cv::Mat RxFrame, RxPC, RxTCW, RxDynamicPC, RxMotionVec;
    unsigned long long int timeRx = readTimeStamp(frameSeqRx);

//    /// time sync module
//    unsigned long long int CurrTS = myAVR->getCurrentAVRFrame_TimeStamp();
//    while( timeRx < CurrTS || timeRx==0){
//        frameSeqRx++;
//        cout << "reading frame: "  << frameSeqRx;
//        cout << "rx ts: " << timeRx << ", cur ts: " << CurrTS << endl;
//        timeRx = readTimeStamp(frameSeqRx);
//    }
//    if (timeRx - CurrTS > 300){
//        cout << "current frame lagging behind\n";
//        cout << "rx ts: " << timeRx << ", cur ts: " << CurrTS << endl;
////        return;
//    }


    readPC(frameSeqRx,RxPC);
    if (RxPC.empty()) {
        cerr << "VCluster::TXRX() can't load rx PC " << frameSeqRx << endl;
//        return;
    }
    readTcw(frameSeqRx,RxTCW);
    if (RxTCW.empty()) {
        cerr << "VCluster::TXRX() can't load tcw " << frameSeqRx << endl;
//        return;
    }
    /// basic info complete
    myAVR->RxBuffer.put_PC(RxPC);
    myAVR->RxBuffer.put_TCW(RxTCW);

    /// not care whether others are atomic fow now
    readFrame(frameSeqRx, RxFrame);
    if (RxFrame.empty()){
        cerr << "VCluster::TXRX() can't load rx frame " << frameSeqRx << endl;
//        return;
    }
    myAVR->RxBuffer.put_FrameLeft(RxFrame);
    if (DYNAMICS){
        readDynamicPC(frameSeqRx,RxDynamicPC);
        readLowPassObjectMotionVec(frameSeqRx, RxMotionVec);
        if (RxDynamicPC.empty()){
            cerr << "VCluster::TXRX() can't load rx dynamic frame " << frameSeqRx << endl;
//            return;
        }
        myAVR->RxBuffer.put_dynamicPC(RxDynamicPC);
        myAVR->RxBuffer.put_MotionVec(RxMotionVec);
    }


    myAVR->RxBuffer.finishReceivingFrame();
}