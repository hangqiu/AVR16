//
// Created by nsl on 9/23/16.
//

#include "ObjSender.hpp"
#include "stdafx.hpp"
#include "ObjSenderHandler.hpp"
#include "mySocket.hpp"

using namespace std;

using namespace web;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;

ObjSender::ObjSender(AugmentedVR *myAVR, string commPath) : myAVR(myAVR), commPath(commPath) {
//    initCPPREST();
    initMySocket();
    sendFlag = false;

}

ObjSender::~ObjSender() {
//    TcwFile.release();
//
//    PCFile.release();
    g_httpHandler->close().wait();
    delete &mSock;
}
void sigchld_handler(int s) {
    while (waitpid(-1, NULL, WNOHANG) > 0)
        ;
}
void ObjSender::initMySocket(){
    ///init my own socket, no need for http, too slow
    mSock.Bind(MyPort.c_str());
    mSock.Listen();
    struct sigaction sa;
    sa.sa_handler = sigchld_handler; // reap all dead processes
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGCHLD, &sa, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }
    mSock.Accept();
//    thread* streamer = new std::thread(&ObjSender::StreamPointCloud,this);
}




void ObjSender::StreamPointCloud(){
//    int LastAckedFrameID=-1;
//    if (LastAckedFrameID<FRAME_ID-3){
        cv::FileStorage fs;
        AVRFrame Frame;
        myAVR->getLastAVRFrame(Frame);

        int txSize;
        string seq = std::to_string(Frame.frameSeq);
        txSize = seq.length()+1;
        cout << txSize << endl;
        int bufsize =15;
        string txSizeString = std::to_string(txSize);
        mSock.Send(txSizeString.c_str(), bufsize);
        mSock.Send(seq.c_str(), txSize);

        string ts = std::to_string(Frame.frameTS);
        txSize = ts.length()+1;
        cout << txSize << endl;
        string txSizeStr = std::to_string(txSize);
        mSock.Send(txSizeStr.c_str(), bufsize);
        mSock.Send(ts.c_str(), txSize);

        txSize = Frame.CamMotionMat.total()*Frame.CamMotionMat.elemSize();
        cout << txSize << endl;
        string txsizestr = std::to_string(txSize);
        cout << Frame.CamMotionMat.type() << endl;
        mSock.Send(txsizestr.c_str(), bufsize);
        mSock.Send((const char*)Frame.CamMotionMat.data, txSize);

        timeval tFetchEnd,tFetchStart;
        gettimeofday(&tFetchStart,NULL);
        cout << "TimeStamp Start: " << tFetchStart.tv_sec << "sec" << tFetchStart.tv_usec << "usec";
//        cout << "TimeStamp Start: " << double(tFetchEnd.tv_sec)*1000 + double(tFetchEnd.tv_usec) / 1000 << "ms: ";

        txSize = Frame.pointcloud.total()*Frame.pointcloud.elemSize();
        cout << Frame.pointcloud.type()<< endl;
        cout << txSize << endl;
        mSock.Send(std::to_string(txSize).c_str(), bufsize);
        mSock.Send((const char*)Frame.pointcloud.data, txSize);

        gettimeofday(&tFetchEnd,NULL);
        cout << "TimeStamp End: " << tFetchEnd.tv_sec << "sec" << tFetchEnd.tv_usec << "usec";
//        cout << "TimeStamp End: " << double(tFetchEnd.tv_sec)*1000 + double(tFetchEnd.tv_usec) / 1000 << "ms: ";
        cout << "PC TX: " << double(tFetchEnd.tv_sec-tFetchStart.tv_sec)*1000 + double(tFetchEnd.tv_usec-tFetchStart.tv_usec) / 1000 << "ms" << endl;

//    }
}

void ObjSender::initCPPREST(){
    //init http server, need to be run in separate thread
    utility::string_t port = U(MyPort.c_str());

    utility::string_t address = U(MyAddress.c_str());
    address.append(port);

    uri_builder uri(address);


    auto addr = uri.to_uri().to_string();
    g_httpHandler = std::unique_ptr<ObjSenderHandler>(new ObjSenderHandler(addr));
    g_httpHandler->open().wait();

    ucout << utility::string_t(U("Listening for requests at: ")) << addr << std::endl;
}


void ObjSender::writeFrameInSeparateFile(){
//    writeCurrentFrame();
    writeCurrentTcw();
    writeCurrentTimeStamp();
    writePC();
//    writeDynamicPC();
//    writeObjectMotionVec();
//    writeLowPassObjectMotionVec();
}


void ObjSender::PrepFileStorageBuffer(cv::FileStorage & buf){
    AVRFrame Frame;
    myAVR->getLastAVRFrame(Frame);
    cv::Mat tcw;
    int frameseq = Frame.frameSeq;
    int ts = int(Frame.frameTS);
    Frame.CamMotionMat.copyTo(tcw);
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/%s_%s_%s_%d.yml", myAVR->CamId,
            FRAME.c_str(), TCW.c_str(), TIMESTAMP.c_str(), Frame.frameSeq);
    buf.open(commPath+tmpstr, cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);/// when memory is specified, filename is just the format
    buf << SEQNO.c_str() << frameseq;
//    SenderBuffer << FRAME << currFrame.FrameLeft;
    buf << TCW.c_str() << tcw;
    buf << TIMESTAMP.c_str() << ts;
//    SenderBuffer << PC << currFrame.pointcloud;
}

void ObjSender::PrepSenderBuffer(){
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/%s_%s_%s_%d.yml", myAVR->CamId,
            FRAME.c_str(), TCW.c_str(), TIMESTAMP.c_str(), currFrame.frameSeq);
    SenderBuffer.open(commPath+tmpstr, cv::FileStorage::WRITE + cv::FileStorage::MEMORY);/// when memory is specified, filename is just the format
    SenderBuffer << SEQNO << currFrame.frameSeq;
//    SenderBuffer << FRAME << currFrame.FrameLeft;
    SenderBuffer << TCW << currFrame.CamMotionMat;
    SenderBuffer << TIMESTAMP << (int)currFrame.frameTS;
//    SenderBuffer << PC << currFrame.pointcloud;
}

char* ObjSender::writeFullFrame_PC_TCW_Time_Memory(){
    char tmpstr[100];
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    sprintf(tmpstr, "/cam%d/%s_%s_%s_%d.yml", myAVR->CamId,
            FRAME.c_str(), TCW.c_str(), TIMESTAMP.c_str(), currFrame.frameSeq);
    SenderBuffer.open(commPath+tmpstr, cv::FileStorage::WRITE + cv::FileStorage::MEMORY);/// when memory is specified, filename is just the format
    SenderBuffer << FRAME << currFrame.FrameLeft;
    SenderBuffer << TCW << currFrame.CamMotionMat;
    SenderBuffer << TIMESTAMP << (int)currFrame.frameTS;
    SenderBuffer << PC << currFrame.pointcloud;
//    WholeFrame << DYNAMICPC << currFrame.DynamicPC;
//    WholeFrame << MOTIONVEC <<  currFrame.ObjectMotionVec;
//    WholeFrame << LOWPASSMOTIONVEC <<  currFrame.LowPass_ObjectMotionVec;
//    WholeFrame.release();
    return tmpstr;
}

char* ObjSender::writeFullFrame_PC_TCW_Time(){
    char tmpstr[100];
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    sprintf(tmpstr, "/cam%d/%s_%s_%s_%d.yml", myAVR->CamId,
            FRAME.c_str(), TCW.c_str(), TIMESTAMP.c_str(), currFrame.frameSeq);
    WholeFrame.open(commPath+tmpstr, cv::FileStorage::WRITE);
    WholeFrame << FRAME << currFrame.FrameLeft;
    WholeFrame << TCW << currFrame.CamMotionMat;
    WholeFrame << TIMESTAMP << (int)currFrame.frameTS;
    WholeFrame << PC << currFrame.pointcloud;
//    WholeFrame << DYNAMICPC << currFrame.DynamicPC;
//    WholeFrame << MOTIONVEC <<  currFrame.ObjectMotionVec;
//    WholeFrame << LOWPASSMOTIONVEC <<  currFrame.LowPass_ObjectMotionVec;
    WholeFrame.release();
    return tmpstr;
}

char* ObjSender::writePC_TCW_Time(){
    char tmpstr[100];
    AVRFrame currFrame ;
    myAVR->getCurrentAVRFrame(currFrame);
    sprintf(tmpstr, "/cam%d/%s_%s_%d.yml", myAVR->CamId,
            TCW.c_str(), TIMESTAMP.c_str(), currFrame.frameSeq);
    WholeFrame.open(commPath+tmpstr, cv::FileStorage::WRITE);
//    WholeFrame << FRAME << currFrame.FrameLeft;
    WholeFrame << TCW << currFrame.CamMotionMat;
    WholeFrame << TIMESTAMP << (int)currFrame.frameTS;
    WholeFrame << PC << currFrame.pointcloud;
//    WholeFrame << DYNAMICPC << currFrame.DynamicPC;
//    WholeFrame << MOTIONVEC <<  currFrame.ObjectMotionVec;
//    WholeFrame << LOWPASSMOTIONVEC <<  currFrame.LowPass_ObjectMotionVec;
    WholeFrame.release();
    return tmpstr;
}

char* ObjSender::writeWholeFrameWithFullMetaData(){
    char tmpstr[100];
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    sprintf(tmpstr, "/cam%d/Frame%d.yml", myAVR->CamId, currFrame.frameSeq);
    WholeFrame.open(commPath+tmpstr, cv::FileStorage::WRITE);
//    WholeFrame << FRAME << currFrame.FrameLeft;
    WholeFrame << TCW << currFrame.CamMotionMat;
//    WholeFrame << TIMESTAMP << (int)currFrame.frameTS;
    WholeFrame << PC << currFrame.pointcloud;
//    WholeFrame << DYNAMICPC << currFrame.DynamicPC;
//    WholeFrame << MOTIONVEC <<  currFrame.ObjectMotionVec;
//    WholeFrame << LOWPASSMOTIONVEC <<  currFrame.LowPass_ObjectMotionVec;
    WholeFrame.release();
    return tmpstr;
}

void ObjSender::writeCurrentFrame(){
    char tmpstr[100];
    AVRFrame currFrame ;
    myAVR->getCurrentAVRFrame(currFrame);
    sprintf(tmpstr, "/cam%d/Frame%d.yml", myAVR->CamId, currFrame.frameSeq);
    TcwFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    TcwFile << FRAME << currFrame.FrameLeft;
    TcwFile.release();
}

void ObjSender::writeCurrentTcw(){
    char tmpstr[100];
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    sprintf(tmpstr, "/cam%d/TcwFrame%d.yml", myAVR->CamId, currFrame.frameSeq);
    TcwFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    TcwFile << TCW << currFrame.CamMotionMat;
    TcwFile.release();
}

void ObjSender::writeCurrentTimeStamp(){
//    char tmpstr[50];
//    sprintf(tmpstr, "timeFrame%d",myAVR->frameSeq);
//    TcwFile << tmpstr << (int)myAVR->frameTS; //TODO cannot support long??
    AVRFrame currFrame;
    myAVR->getCurrentAVRFrame(currFrame);
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/timeFrame%d.yml", myAVR->CamId, currFrame.frameSeq);
    TcwFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    double ts = (double)currFrame.frameTS;
    TcwFile << TIMESTAMP << ts;
    TcwFile.release();
}


void ObjSender::writePC(){
//    char tmpstr[50];
//    sprintf(tmpstr, "fullPCFrame%d",myAVR->frameSeq);
//    PCFile << tmpstr << myAVR->pointcloud;
    AVRFrame currFrame; myAVR->getCurrentAVRFrame(currFrame);
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/fullPCFrame%d.yml", myAVR->CamId, currFrame.frameSeq);
    PCFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    PCFile << PC << currFrame.pointcloud;
    PCFile.release();
}

void ObjSender::writeDynamicPC(){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    AVRFrame currFrame; myAVR->getCurrentAVRFrame(currFrame);
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/dynamicPCFrame%d.yml", myAVR->CamId, currFrame.frameSeq);
    dynamicPCFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    //TODO investigate 0
    dynamicPCFile << DYNAMICPC << currFrame.DynamicPC;
//    dynamicPCFile << DYNAMICPC << currFrame.DynamicPC;
    dynamicPCFile.release();
}



void ObjSender::writeObjectMotionVec(){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    AVRFrame currFrame; myAVR->getCurrentAVRFrame(currFrame);
    cout << "sending motion vec\n";
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/objectMotionVec%d.yml", myAVR->CamId, currFrame.frameSeq);
    motionFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    //TODO investigate 0
    motionFile << MOTIONVEC <<  currFrame.ObjectMotionVec;
    cout << currFrame.ObjectMotionVec << endl;
    motionFile.release();
}

void ObjSender::writeLowPassObjectMotionVec(){
//    char tmpstr[50];
//    sprintf(tmpstr, "dynamicPCFrame%d",myAVR->frameSeq);
//    dynamicPCFile << tmpstr << myAVR->DynamicPC;
    AVRFrame currFrame; myAVR->getCurrentAVRFrame(currFrame);
    cout << "sending low pass motion vec\n";
    char tmpstr[100];
    sprintf(tmpstr, "/cam%d/LowPassObjectMotionVec%d.yml", myAVR->CamId, currFrame.frameSeq);
    motionFile.open(commPath+tmpstr, cv::FileStorage::WRITE);
    motionFile << LOWPASSMOTIONVEC <<  currFrame.LowPass_ObjectMotionVec;
    cout << currFrame.LowPass_ObjectMotionVec << endl;
    motionFile.release();
}

//void ObjSender::logFrame(){
//    char tmpstr[100];
//    sprintf(tmpstr, "/cam%d/FrameInfo.txt", myAVR->CamId);
//    logFile.open(commPath+tmpstr, fstream::app);
//    logFile << "Frame " << myAVR->getCurrentAVRFrame()->frameSeq << ": TS: "<< myAVR->getCurrentAVRFrame()->ZEDTS
//            << ": Obj MotionVec: " << myAVR->ObjectMotionVec << ": dist: " << norm(myAVR->ObjectMotionVec)
//            << ": LP_MotionVec: " << myAVR-> Log_LowPassMotionVec << ": dist: " << norm(myAVR->Log_LowPassMotionVec) << endl;
//    logFile.close();
//}