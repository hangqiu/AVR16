//
// Created by hang on 11/22/17.
//

#include "PingpongRxBuffer.hpp"

PingpongRxBuffer::PingpongRxBuffer(){
    NextFrameBufferPtr->mLock.lock();
}


RxFrame* PingpongRxBuffer::getCurrentRxFrame(){
    CurrFrame.setFrom(*CurrFrameBufferPtr);
    return &CurrFrame;
}

//void PingpongRxBuffer::beginReceiving(){
//    NextFrameBufferPtr->mLock.lock();
//}

void PingpongRxBuffer::put_Seq(int seq){
    NextFrameBufferPtr->RxSeq = seq;
}
void PingpongRxBuffer::put_TimeStamp(int ts){
    NextFrameBufferPtr->RxTimeStamp = ts;
}
void PingpongRxBuffer::put_TimeStamp_ZEDTS(unsigned long long ts){
    NextFrameBufferPtr->RxTimeStamp = ts;
}
void PingpongRxBuffer::put_FrameLeft(cv::Mat & FrameLeft){
    FrameLeft.copyTo(NextFrameBufferPtr->RxFrameLeft);
}
void PingpongRxBuffer::put_TCW(cv::Mat & TCW){
    TCW.copyTo(NextFrameBufferPtr->RxTCW);
}
void PingpongRxBuffer::put_PC(cv::Mat & PC){
    PC.copyTo(NextFrameBufferPtr->RxPC);
}
void PingpongRxBuffer::put_MotionVec(cv::Mat & MV){
    MV.copyTo(NextFrameBufferPtr->RxMotionVec);
}
void PingpongRxBuffer::put_dynamicPC(cv::Mat & dPC){
    dPC.copyTo(NextFrameBufferPtr->RxDynamicPC);
}

void PingpongRxBuffer::finishReceivingFrame(){
    NextFrameBufferPtr->mLock.unlock();
    RxFrame* tmp = NextFrameBufferPtr;
    NextFrameBufferPtr = CurrFrameBufferPtr;
    CurrFrameBufferPtr = tmp;
    NextFrameBufferPtr->mLock.lock();
}
