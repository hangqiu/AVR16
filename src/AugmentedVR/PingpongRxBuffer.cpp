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
void PingpongRxBuffer::put_TimeStamp(unsigned long long ts){
    NextFrameBufferPtr->RxTimeStamp = ts;
}
void PingpongRxBuffer::put_TimeStamp_ZEDTS(unsigned long long ts){
    NextFrameBufferPtr->RxTimeStamp_ZEDTS = ts;
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
void PingpongRxBuffer::put_LowPassMotionVec(cv::Mat & MV){
    MV.copyTo(NextFrameBufferPtr->RxLowPassMotionVec);
}
void PingpongRxBuffer::put_dynamicPC(cv::Mat & dPC){
    dPC.copyTo(NextFrameBufferPtr->RxDynamicPC);
}

void PingpongRxBuffer::finishReceivingFrame(){
    CurrFrameBufferPtr->mLock.lock();
//    NextFrameBufferPtr->mLock.unlock();
    RxFrame* tmp = NextFrameBufferPtr;
    NextFrameBufferPtr = CurrFrameBufferPtr;
    CurrFrameBufferPtr = tmp;
//    NextFrameBufferPtr->mLock.lock();
    CurrFrameBufferPtr->mLock.unlock();
}

void PingpongRxBuffer::integrateCurrentFrameRxMotionVec(int RXMV_seq, unsigned long long RXMV_ZEDTS, cv::Mat &MV){
    /// need to reacquire curr lock to update the deltas
    CurrFrameBufferPtr->mLock.lock();
    CurrFrameBufferPtr->RxMotionVecSeq.push_back(RXMV_seq);
    CurrFrameBufferPtr->RxMotionVec_ZEDTS.push_back(RXMV_ZEDTS);
    CurrFrameBufferPtr->RxMotionVec = cv::Mat(CurrFrameBufferPtr->RxMotionVec + MV) ;
    CurrFrameBufferPtr->mLock.unlock();
}