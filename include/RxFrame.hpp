//
// Created by hang on 11/22/17.
//

#ifndef PROJECT_RXFRAME_H
#define PROJECT_RXFRAME_H


#include <opencv2/core/mat.hpp>
#include <mutex>

class RxFrame {
private:
public:
    int RxSeq;
    int RxTimeStamp;
    unsigned long long RxTimeStamp_ZEDTS;
    cv::Mat RxFrameLeft;
    cv::Mat RxTCW;
    cv::Mat RxPC;
    cv::Mat RxMotionVec; // motion vector of object received
    cv::Mat transRxPC;
    cv::Mat RxDynamicPC; // transfromed txceived PC
    cv::Mat transRxDynamicPC; // transfromed txceived PC
    std::mutex mLock;

public:
    RxFrame();
    ~RxFrame();
    void setFrom(RxFrame& Frame);
};


#endif //PROJECT_RXFRAME_H
