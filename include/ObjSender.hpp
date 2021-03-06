//
// Created by nsl on 9/23/16.
//

#ifndef PROJECT_OBJSENDER_H
#define PROJECT_OBJSENDER_H


#include <opencv2/core/persistence.hpp>
#include "AugmentedVR.hpp"
#include "ObjSenderHandler.hpp"
#include "mySocket.hpp"

/*<< Defines the server. >>*/
//struct hello_world;
//typedef http::server<hello_world> server;


class SendingThreadQueue{
public:

    thread * mThread_ptr=NULL;
    SendingThreadQueue * mNextThread = NULL;

    SendingThreadQueue(thread* T){
        mThread_ptr = T;
    }
};

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

    cv::FileStorage SenderBuffer;
    string commPath;

    std::unique_ptr<ObjSenderHandler> g_httpHandler;

    mySocket mSock;

    bool sendFlag;

//    bool V2VDEBUG = false;

    int bufsize =15;
    size_t txSize;


    thread *txstream;

    mutex SendingFlag_Lock;
    bool Sending = false;

    mutex SocketLock;

    SendingThreadQueue* mThreadQ =NULL;
    thread *clearSendingQueue;
    mutex QLock;

    bool ObjSenderEnd = false;
    int SilentCount = 0;
    int SilentMax = 0;

public:

    AVRFrame FrameToSend;


    ObjSender(AugmentedVR *myAVR, string commPath);

    ~ObjSender();

    void initMySocket();
    void initCPPREST();

    void httpServerRun(string address, string port);


    void PrepSenderBuffer();
    void PrepFileStorageBuffer(cv::FileStorage & buf);
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
    char* writeFullFrame_PC_TCW_Time_Memory();
    char* writePC_TCW_Time();

    void writeFrameInSeparateFile();

    /// in use
    void Stream();
    void StreamMotionVec();
    void StreamPointCloud();
    void StreamPointCloud_Async();
    void StreamStreamType(string StreamType);
    void StreamPointCloud_FrameSeq(AVRFrame & Frame);
    void StreamPointCloud_TimeStamp_FrameTS(AVRFrame &Frame);
    void StreamPointCloud_TimeStamp_ZEDTS(AVRFrame &Frame);
    void StreamPointCloud_TCW(AVRFrame & Frame);
    void StreamPointCloud_PC(AVRFrame & Frame);
    void StreamPointCloud_DynamicPC(AVRFrame & Frame);
    void StreamPointCloud_Frame(AVRFrame & Frame);
    void StreamPointCloud_ObjectMotionVec(AVRFrame &Frame);
    void StreamPointCloud_LowPass_ObjectMotionVec(AVRFrame &Frame);

    void SetSendingFlagTrue();
    void SetSendingFlagFalse();
    bool IsSending();

    void CheckSendingQueue();
    void addMotionVecToSenderQueue();

    void WaitACK_PCFrame();
};


#endif //PROJECT_OBJSENDER_H
