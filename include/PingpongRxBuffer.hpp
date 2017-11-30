//
// Created by hang on 11/22/17.
//

#ifndef PROJECT_PINGPONGRXBUFFER_H
#define PROJECT_PINGPONGRXBUFFER_H


#include "RxFrame.hpp"

class PingpongRxBuffer {
public:
    RxFrame FrameA;
    RxFrame FrameB;

    RxFrame* CurrFrameBufferPtr = &FrameA;
    RxFrame* NextFrameBufferPtr = &FrameB;

    RxFrame CurrFrame;

    RxFrame * getCurrentRxFrame();

    PingpongRxBuffer();
//    void beginReceiving();
    void put_Seq(int seq);
    void put_TimeStamp(int ts);
    void put_TimeStamp_ZEDTS(unsigned long long ts);
    void put_FrameLeft(cv::Mat & FrameLeft);
    void put_TCW(cv::Mat & TCW);
    void put_PC(cv::Mat & PC);
    void put_MotionVec(cv::Mat & MV);
    void put_dynamicPC(cv::Mat & dPC);
    void finishReceivingFrame();
};


#endif //PROJECT_PINGPONGRXBUFFER_H
