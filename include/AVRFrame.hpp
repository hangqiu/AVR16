//
// Created by hang on 9/26/17.
//

#ifndef PROJECT_AVRFRAME_H
#define PROJECT_AVRFRAME_H


#include <opencv2/core/mat.hpp>
#include <sl/Core.hpp>

class AVRFrame {
private:
    mutex FrameLock;
public:
    AVRFrame();
//    AVRFrame(const AVRFrame &frame);
    ~AVRFrame();

//    AVRFrame(long ZEDTS, long FrameTS, int frameSeq, cv::Mat &FrameLeft, cv::Mat &FrameRight, cv::Mat &FrameLeftGray,
//                 cv::Mat &FrameRightGray, cv::Mat &pointcloud, cv::Mat &PC_noColor, sl::Mat pointcloud_sl);

    void lockFrame();

    void unlockFrame();

    void setFrom(AVRFrame& frame);

    void detectNewFeature();

    void CacheExistingFeature();

    void updateFeature();

    bool isEmpty();



    //////////////////////////////////basic info/////////////////
    // Frame metaData
    unsigned long long int frameTS;
    unsigned long long int ZEDTS;
    int frameSeq;

    // ZED Data
    cv::Mat FrameLeft;
    cv::Mat FrameRight;
    cv::Mat FrameLeftGray;
    cv::Mat FrameRightGray;

    cv::Mat pointcloud;
//    sl::Mat pointcloud_sl;

    cv::Mat PC_noColor;
////////////////////////////////////optional////////////////////////
    // ORB_SLAM data
    cv::Mat CamMotionMat; // Tcw // (4,4,CV_32F) rotation mat and translation mat Tcw (Rcw, tcw) in ORBSLAM

    cv::Mat TranslationMat_Curr2CacheHead; // tlc
    cv::Mat RotationMat_Curr2CacheHead; // Rlc
    cv::Mat TransformationMat_Curr2CacheHead; // Tlc

    cv::Mat transformedPointcloud;

    cv::Mat DynamicFrame; // the corresponding frame after PC motion filtering, to double confirm the filtering works out correctly.
    // this is the motion mask appilied on FrameLeft in this struct, which means a hitoric frame, not current frame.
    cv::Mat DynamicPC; // filtered by motion displacement threshold
    cv::Mat MotionMask;

    cv::Mat PCDisplacement;
    cv::Mat PCMotionVec; // matrix of pc motion

    cv::Mat ObjectMotionVec; // matrix of average object motion vec
    cv::Mat FilteredObjectMotionVec; // matrix of average object motion vec
    cv::Mat LowPass_ObjectMotionVec; // matrix of average object motion vec
    cv::Mat LowPass_FilteredObjectMotionVec;

    std::vector<cv::Point2f> keypoints;
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<cv::Point2f> tracked_keypoints;
    std::vector<uchar> tracked_status;
    std::vector<float> tracked_error;

//    vector<Point2f> keypoints_cache;

//    landmark_matchinfo matched_scene;
    cv::Mat sceneTransformMat_Curr2Last;
    cv::Mat sceneTransformMat_Curr2CacheHead;

    void getPointCloud(cv::Mat & ret);

    int getFrameTS();

    void updateMotionData();

    void updatePCDisplacementFromMotionVec();

    void updatePCMotionVecFromPCandTransformedPC();

    void tranformPointCloud_via_TransformationMat(cv::Mat &HomographyTransMat_Curr2CacheHead);

    void updateMotionMask_via_ThresholdingPCDisplacement();

    void removeMotionMaskHighLow();

    void updateDynamicsFromMotionMask();

    void MotionAnalysis();
};


#endif //PROJECT_AVRFRAME_H
