//
// Created by hang on 11/22/17.
//

#include "RxFrame.hpp"
RxFrame::RxFrame(){

}
RxFrame::~RxFrame(){

}

void RxFrame::setFrom(RxFrame& Frame){
    Frame.mLock.lock();
    mLock.lock();

    RxSeq = Frame.RxSeq;
    RxTimeStamp = Frame.RxTimeStamp;
    RxTimeStamp_ZEDTS = Frame.RxTimeStamp_ZEDTS;
    Frame.RxFrameLeft.copyTo(RxFrameLeft);
    Frame.RxTCW.copyTo(RxTCW);
    Frame.RxPC.copyTo(RxPC);
    Frame.RxMotionVec.copyTo(RxMotionVec);
    Frame.transRxPC.copyTo(transRxPC);
    Frame.RxDynamicPC.copyTo(RxDynamicPC);
    Frame.transRxDynamicPC.copyTo(transRxDynamicPC);

    mLock.unlock();
    Frame.mLock.unlock();
}