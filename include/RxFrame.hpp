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
    unsigned long long RxTimeStamp;
    unsigned long long RxTimeStamp_ZEDTS;
    cv::Mat RxFrameLeft;
    cv::Mat RxTCW;
    cv::Mat RxPC;
    cv::Mat RxLowPassMotionVec; // Low pass filterd motion vector of object received
    cv::Mat transRxPC;
    cv::Mat RxDynamicPC; // transfromed txceived PC
    cv::Mat transRxDynamicPC; // transfromed txceived PC
    std::mutex mLock;

    /// keep only the last received delta seq, ts, and integrated total delta.
    /// TODO vector has memory leak...
    std::vector<int> RxMotionVecSeq;
    std::vector<unsigned long long> RxMotionVec_ZEDTS;
    cv::Mat RxMotionVec = cv::Mat(1,1,CV_32FC3, cv::Scalar(0.,0.,0.)); // integrated delta of object received

public:
    RxFrame();
    ~RxFrame();
    void setFrom(RxFrame& Frame);
};


#endif //PROJECT_RXFRAME_H
