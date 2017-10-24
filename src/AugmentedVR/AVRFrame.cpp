//
// Created by hang on 9/26/17.
//

#include <opencv2/imgproc.hpp>
#include <include/globals.hpp>
#include <include/UtilsPC.hpp>
#include "AVRFrame.hpp"

AVRFrame::AVRFrame(){
    FrameLeft = cv::Mat();
    FrameRight = cv::Mat();
    FrameLeftGray = cv::Mat();
    FrameRightGray = cv::Mat();
    pointcloud =  cv::Mat();
    PC_noColor = cv::Mat();
    CamMotionMat= cv::Mat();
    TranslationMat_Curr2CacheHead= cv::Mat(); // tlc
    RotationMat_Curr2CacheHead= cv::Mat(); // Rlc
    transformedPointcloud= cv::Mat();
    DynamicFrame= cv::Mat();
    DynamicPC= cv::Mat(); // filtered by motion displacement threshold
    MotionMask= cv::Mat();
    PCDisplacement= cv::Mat();
    PCMotionVec= cv::Mat(); // matrix of pc motion
    ObjectMotionVec= cv::Mat(); // matrix of average object motion vec
    LowPass_ObjectMotionVec= cv::Mat(); // matrix of average object motion vec
    sceneTransformMat= cv::Mat();
}
AVRFrame::~AVRFrame(){

}

//AVRFrame::AVRFrame(long ZEDTS, long FrameTS, int frameSeq,
//                   cv::Mat& FrameLeft, cv::Mat& FrameRight, cv::Mat& FrameLeftGray, cv::Mat& FrameRightGray,
//                   cv::Mat& pointcloud, cv::Mat& PC_noColor,
//                   sl::Mat pointcloud_sl=sl::Mat()){
//    AVRFrame::ZEDTS = ZEDTS;
//    AVRFrame::frameTS = FrameTS;
//    AVRFrame::frameSeq = frameSeq;
//
//    FrameLeft.copyTo(AVRFrame::FrameLeft);
//    FrameLeftGray.copyTo(AVRFrame::FrameLeftGray);
//    FrameRight.copyTo(AVRFrame::FrameRight);
//    FrameRightGray.copyTo(AVRFrame::FrameRightGray);
//
//    pointcloud.copyTo(AVRFrame::pointcloud);
//    PC_noColor.copyTo(AVRFrame::PC_noColor);
////    AVRFrame::pointcloud_sl.setFrom(pointcloud_sl);//TODO
//
//    PCDisplacement = cv::Mat(height,width, CV_32F, cv::Scalar(255.0));
//}

//AVRFrame::AVRFrame(const AVRFrame& frame) {
//
//    ZEDTS = frame.ZEDTS;
//    frameTS = frame.frameTS;
//    frameSeq = frame.frameSeq;
//    frame.FrameLeft.copyTo(FrameLeft);
//    frame.FrameRight.copyTo(FrameRight);
//    frame.FrameLeftGray.copyTo(FrameLeftGray);
//    frame.FrameRightGray.copyTo(FrameRightGray);
//    frame.pointcloud.copyTo(pointcloud);
////    frame.pointcloud_sl.copyTo(pointcloud_sl);
//    frame.PC_noColor.copyTo(PC_noColor);
//
//    frame.CamMotionMat.copyTo(CamMotionMat);
//    frame.DynamicFrame.copyTo(DynamicFrame);
//    frame.DynamicPC.copyTo(DynamicPC);
//    frame.MotionMask.copyTo(MotionMask);
//    frame.PCMotionVec.copyTo(PCMotionVec);
//    frame.ObjectMotionVec.copyTo(ObjectMotionVec);
//    frame.LowPass_ObjectMotionVec.copyTo(LowPass_ObjectMotionVec);
//
//    keypoints = frame.keypoints;
//    status = frame.status;
//    error = frame.error;
//    tracked_keypoints = frame.tracked_keypoints;
//    tracked_status = frame.tracked_status;
//    tracked_error = frame.tracked_error;
//
//    frame.sceneTransformMat.copyTo(sceneTransformMat);
//}

void AVRFrame::setFrom(AVRFrame& frame) {
    frame.FrameLock.lock();
    FrameLock.lock();

    ZEDTS = frame.ZEDTS;
    frameTS = frame.frameTS;
    frameSeq = frame.frameSeq;
    frame.FrameLeft.copyTo(FrameLeft);
    frame.FrameRight.copyTo(FrameRight);
    frame.FrameLeftGray.copyTo(FrameLeftGray);
    frame.FrameRightGray.copyTo(FrameRightGray);
    frame.pointcloud.copyTo(pointcloud);
//    frame.pointcloud_sl.copyTo(pointcloud_sl);
    frame.PC_noColor.copyTo(PC_noColor);

    frame.CamMotionMat.copyTo(CamMotionMat);
    frame.DynamicFrame.copyTo(DynamicFrame);
    frame.DynamicPC.copyTo(DynamicPC);
    frame.MotionMask.copyTo(MotionMask);
    frame.PCMotionVec.copyTo(PCMotionVec);
    frame.ObjectMotionVec.copyTo(ObjectMotionVec);
    frame.LowPass_ObjectMotionVec.copyTo(LowPass_ObjectMotionVec);

    keypoints = frame.keypoints;
    status = frame.status;
    error = frame.error;
    tracked_keypoints = frame.tracked_keypoints;
    tracked_status = frame.tracked_status;
    tracked_error = frame.tracked_error;

    frame.sceneTransformMat.copyTo(sceneTransformMat);

    FrameLock.unlock();
    frame.FrameLock.unlock();
}

bool AVRFrame::isEmpty(){
    return FrameLeft.empty();
}

void AVRFrame::detectNewFeature(){

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    cv::Size subPixWinSize(10,10);

    cv::goodFeaturesToTrack(FrameLeftGray, keypoints, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
    cornerSubPix(FrameLeftGray, keypoints, subPixWinSize, cv::Size(-1,-1), termcrit);

}

void AVRFrame::CacheExistingFeature(){
    tracked_keypoints = keypoints;
    tracked_status = status;
    tracked_error = error;
}

void AVRFrame::updateFeature(){
    FrameLock.lock();
    CacheExistingFeature();
    detectNewFeature();
    FrameLock.unlock();
}


void AVRFrame::MotionAnalysis(){
//    FrameLock.lock();
    updatePCDisplacementFromMotionVec();
    updateMotionMask_via_ThresholdingPCDisplacement();
    removeMotionMaskHighLow();
    updateDynamicsFromMotionMask();
//    FrameLock.unlock();
}

void AVRFrame::updatePCDisplacementFromMotionVec(){
    assert(!PCMotionVec.empty());
    PCDisplacement = MatPerElementNorm(PCMotionVec);

    if (DEBUG && VISUAL){
        char tmp[50];
        sprintf(tmp, "Displacement %d", frameSeq);
        imshow(tmp, PCDisplacement);
        cv::setMouseCallback(tmp, onMouseCallback_DisplayDisplacement, &PCDisplacement);
    }
}

void AVRFrame::updateMotionMask_via_ThresholdingPCDisplacement(){
    assert(!PCDisplacement.empty());

    // THRESHOLDING TO GET THE MASK OF DYNAMIC OBJECTS
    float motionThreshold = MOTIONTHRESH_PERPIXEL*ZEDCACHESIZE; //in meters (0.1-0.15 per frame )
    // tried GPU version of it, slower....
    // store the mask in current frame...
    cv::threshold( PCDisplacement, MotionMask, motionThreshold, 255, cv::THRESH_BINARY);

    if (DEBUG>1) cout << "MotionMask: " << MotionMask << endl;

    MotionMask.convertTo(MotionMask,CV_8U);

}

void AVRFrame::removeMotionMaskHighLow(){
    assert(!MotionMask.empty());
    //    Remove_HighLow
    cv::Mat tmpMask,tmpPC;
    extractChannel(pointcloud,tmpPC,1);

    cv::threshold( tmpPC, tmpMask, HEIGHT_THRESH, 255, cv::THRESH_BINARY_INV);
    tmpMask.convertTo(tmpMask,CV_8U);
    MotionMask &= tmpMask;
}

void AVRFrame::updateDynamicsFromMotionMask(){
    assert(!MotionMask.empty());
    if (DEBUG>1) cout << MotionMask.depth() << MotionMask.channels();

    pointcloud.copyTo(DynamicPC,MotionMask);
    FrameLeft.copyTo(DynamicFrame, MotionMask);

    if (DEBUG>1){
        cv::Mat tmpPC;
        PCDisplacement.copyTo(tmpPC,MotionMask);
        cout << "filtered PCDisplacement: \n" << tmpPC << endl;
    }
}


void AVRFrame::tranformPointCloud_via_TransformationMat(){

    // perspective transform to get pixel level pointcloud matching from last frame

//    FrameLock.lock();

    int width = FrameLeft.size().width;
    int height = FrameLeft.size().height;
    cv::Size outputSize(width,height);
#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
#endif
    cv::warpPerspective(pointcloud,transformedPointcloud,sceneTransformMat,outputSize);
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "   CPU warpperspective: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
#endif

    // TODO: use tlc only for now
    // watch out, orbslam and zed have different default coordinate frame positive difrection
    // same x, opposite y,z, not true any more?
    /// TODO: double chck the new SDK
    assert(!TranslationMat_Curr2CacheHead.empty());
    cv::Scalar tlc_vec = cv::Scalar(TranslationMat_Curr2CacheHead.at<float>(0,0),
                                    TranslationMat_Curr2CacheHead.at<float>(1,0),
                                    TranslationMat_Curr2CacheHead.at<float>(2,0),0);
    cv::Mat tlc_mat(height, width, CV_32FC4, tlc_vec);
    transformedPointcloud = transformedPointcloud+tlc_mat;

    if (DEBUG>1){
        cout << "tlc: " << tlc_mat(cv::Rect(dbx,dby,1,1)) << endl;
//                cout << "tlc_mat4: " << tlc_mat_4(cv::Rect(dbx,dby,1,1)) << endl;
        cout << "transformedPC_inLast: " << transformedPointcloud(cv::Rect(dbx,dby,1,1)) << endl;
    }

//    FrameLock.unlock();

}
