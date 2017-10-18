//
// Created by nsl on 9/20/16.
//

#ifndef PROJECT_AUGMENTEDVR_H
#define PROJECT_AUGMENTEDVR_H

//opencv includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"

//ZED Includes
//#include <zed/Mat.hpp>
//#include <zed/Camera.hpp>
//#include <zed/utils/GlobalDefine.hpp>
#include <sl/Camera.hpp>

//ORB_SLAM Includes
#include "../src/ORB_SLAM2/include/System.h"

//AVR includes
#include <include/landmark_matchinfo.hpp>
#include "globals.hpp"
#include "IO.hpp"
#include "FrameCache.hpp"


#include <queue>

using namespace sl;
using namespace std;
//using namespace cv;
//using namespace cv::xfeatures2d;

//struct ZED_DATA{
//    cv::Mat FrameLeft;
//    cv::Mat FrameRight;
//    cv::Mat FrameLeftGray;
//    cv::Mat FrameRightGray;
//
//    cv::Mat pointcloud;
////    sl::Mat pointcloud_sl;
//
//    cv::Mat PC_noColor;
//
//
//    long ZEDTS;
//    long frameTS;
//};
//
//struct STEREO_DATA{
//
//    long frameTS;
//    long ZEDTS;
//    int frameSeq;
//
//    cv::Mat FrameLeft;
//    cv::Mat FrameRight;
//    cv::Mat FrameLeftGray;
//    cv::Mat FrameRightGray;
//
//    cv::Mat pointcloud;
////    sl::Mat pointcloud_sl;
//
//    cv::Mat PC_noColor;
//
//    cv::Mat CamMotionMat; // Tcw // (4,4,CV_32F) rotation mat and translation mat Tcw (Rcw, tcw) in ORBSLAM
//    cv::Mat DynamicFrame; // the corresponding frame after PC motion filtering, to double confirm the filtering works out correctly.
//    // this is the motion mask appilied on FrameLeft in this struct, which means a hitoric frame, not current frame.
//    cv::Mat DynamicPC; // filtered by motion displacement threshold
//    cv::Mat MotionMask;
//
//    cv::Mat PCMotionVec; // matrix of pc motion
//
//    cv::Mat ObjectMotionVec; // matrix of average object motion vec
//    cv::Mat LowPass_ObjectMotionVec; // matrix of average object motion vec
//
//    vector<cv::Point2f> keypoints;
//    vector<cv::Point2f> tracked_keypoints;
//    vector<uchar> tracked_status;
//
////    vector<Point2f> keypoints_cache;
//
////    landmark_matchinfo matched_scene;
//    cv::Mat sceneTransformMat;
//};

class IO;
class AugmentedVR {

public: //TODO change to private and add agetter seter
    int CamId;
    long startTS;
//    long frameTS;
//    long ZEDTS;
    int TotalFrameSeq;

    // ZED data
    // params
//    sl::zed::SENSING_MODE senseMode;
    sl::InitParameters initParameters;
    sl::RuntimeParameters runtimeParameters;

    int width, height;// video input size
    int ZEDConfidence;
//    cv::Mat SbSResult;
//    cv::Mat FrameLeft;
//    cv::Mat FrameRight;
//    cv::Mat FrameLeftGray;
//    cv::Mat FrameRightGray;

    cv::Mat ZED_LRes;
    cv::Mat BufferXYZRGBA;

//    cv::Mat pointcloud_cv;
////    sl::Mat pointcloud_sl;
//    sl::Mat pointcloud_sl_gpu;

//    cv::Mat PC_noColor;

    cv::Mat initPC;

    bool initialized;

//    ZED_DATA NextFrame;
//    ZED_DATA SlamFrame;// intermedieate buffer between zed and slam,for pipeline
//    STEREO_DATA LastFrame;
//
////    vector<cv::Mat> lastpointcloud;
//    vector<STEREO_DATA> lastStereoData;
//    vector<cv::Mat> PCMotionVec;

//    cv::Mat depth_mat;




    // ORB SLAM data
//    cv::Mat CamMotionMat; // Tcw // (4,4,CV_32F) rotation mat and translation mat Tcw (Rcw, tcw) in ORBSLAM
//    cv::Mat lastCamMotionMat; // (4,4,CV_32F)

    cv::Mat tlc; // translation
    cv::Mat Rlc; // rotation

//    cv::Mat sceneTransformMat;
//    cv::Mat transformedPointcloud; // transforming to the coordinate frame of last frame

//    cv::Mat PCDisplacement;
    cv::Mat ObjectMotionVec; // matrix of average object motion vec

    cv::Mat Log_LowPassMotionVec; // after low pass filter TODO: make a log

    cv::Mat RxMotionVec; // motion vector of object received

    cv::Mat RxPC;
    cv::Mat RxDynamicPC; // transfromed txceived PC

    cv::Mat transRxPC;
    cv::Mat transRxDynamicPC; // transfromed txceived PC

    vector<cv::Point2f> keypoints;
    vector<uchar> status;
    vector<float> err;

    vector<cv::Point2f> keypoints_cache;
    vector<uchar> status_cache;
    vector<float> err_cache;

    FrameCache frameCache;
    sl::Camera* mZEDCam;
    ORB_SLAM2::System* mSLAM;
    IO* mIo;

    thread CPU_download;

    void setMSLAM(ORB_SLAM2::System *mSLAM);



public:
//    AugmentedVR(int CamId);
    AugmentedVR(int CamId, sl::InitParameters initParam, sl::RuntimeParameters runtimeParam, int ZEDConfidence);
    ~AugmentedVR();
    void exit();


    int initZEDCam(int startFrameID);

    void initSLAMStereo(string VocFile, string CalibrationFile, bool bReuseMap = false, string mapFile = "default.txt");
    bool PrepareNextFrame();
    bool grabNextZEDFrameOffline();
    void FeedSlamNextFrame();
//    void fetchNUpdateFrameNPointcloud();
    cv::Mat getGrayBBox(cv::Mat img, int select_left, int select_top, int select_width, int select_height);
    landmark_matchinfo find_obj_in_second_scene(cv::Mat img_object, cv::Mat img_scene);

    void filterPCMotionVec();
    void updateLastStereoData();

    void shiftPC(cv::Mat pc1,cv::Scalar vec);


    void trackCam();

    void analyze();

    void subtractCamMotion();

    void calcSceneTransformMat(int idx);
    void calcCamMotion_CacheIdx2Last(int idx);
    void transfromPCtoLastFrameCoord(int idx);

    void PCMotionAnalysis();

    void filterPCMotion();

    void saveCurFrame();

    ORB_SLAM2::System *getMSLAM() const;

    cv::Mat calcRelaCamPos(cv::Mat TcwReceived);

    cv::Mat translateRxPCtoMyFrameCoord(cv::Mat trc, cv::Mat PCReceived);
    cv::Mat transformRxPCtoMyFrameCoord(cv::Mat Trc, cv::Mat PCReceived);

    int getCamId() const;

    long getStartTS() const;

    long getFrameTS() const;

    long getZEDTS() const;

    void updateLastPointCloud();

    cv::Mat calcPCDisplacement(int idx);

    void ObjectMotionAnalysis(int idx);

    void calcOpticalFlow();

    void retrieve_ZED_PC_CPU();

    bool DutyCycling();

    void PC_segmentation();

    void Remove_HighLow();

    void PC_segmentation(int idx);

    void CheckConnection(int tgt_x, int tgt_y, int cur_x, int cur_y, int idx, std::queue<cv::Point2f> & Q, cv::Mat & checkFlag, cv::Mat & inQueue, float motionThreshold);

    void Remove_HighLow(int idx);

    void dead_reckoning_onRxDynamicPC();

    bool loadSlamFrameAsCurrentFrame();

    void detectFeature();

    cv::Mat removeOverlap(cv::Mat transPC);

    void SinkFrames();

    AVRFrame getCurrentAVRFrame();

    bool trackGood();
};


#endif //PROJECT_AUGMENTEDVR_H
