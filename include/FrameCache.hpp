//
// Created by hang on 9/26/17.
//

#ifndef PROJECT_FRAMECACHE_H
#define PROJECT_FRAMECACHE_H


#include <opencv2/core/mat.hpp>
#include "AVRFrame.hpp"


class FrameCache {
public:
    int CacheSize; // must > 4. since simultanesouly processing 4 frames

    unsigned long long int startTS;
    mutex theBigLock;
    AVRFrame NextFrame;
    AVRFrame SlamFrame;// intermedieate buffer between zed and slam,for pipeline
    AVRFrame CurrentFrame;
    AVRFrame LastFrame;
    unsigned long long int LatestZEDTS;
    unsigned long long int LatestFrameTS;
    unsigned long long int CurrentZEDTS;
    unsigned long long int CurrentFrameTS;
//    std::vector<AVRFrame> fifo;
    AVRFrame* fifo;
    int fifoStartIndex;
    int fifoEndIndex;
    bool fullBacklog = false;

public:
    FrameCache();
    FrameCache(int size);
    ~FrameCache();

    void setStartTS(unsigned long long int startTS);

    void LoadNextFrameFromZED(sl::Camera *mZEDCam, int width, int height, int seq);

    void GrabNewZEDFrame(AVRFrame &NewFrame, sl::Camera *mZEDCam, int width, int height, int seq);

    bool NextFrame2SlamFrame();

    bool SlamFrame2CurrentFrame();

    bool FIFOTail2NextFrame();

    bool CurrentFrame2LastFrame();

    void saveCurFrame();

    bool calcOpticalFlow_Current2Last();

    void FindHomographyMatrix_Curr2Last();

    void FindHomographyMatrix_Curr2CacheHead();

    void FindHomographyMatrix_A2B(AVRFrame&A,AVRFrame&B, cv::Mat &ret);

    void opticalFlowTrack();

    bool FullBacklogAfterSLAM();

    void updatePCDisplacement_Last2FIFOHead();

    void SinkFrames();

    bool updateTransformationMatrix_Curr2CacheHead();

    bool tranformPointCloud_Curr2CacheHead();

    void updateMotionData_Curr2CacheHead();

    void updateCurrFrameMotionVec_Curr2CacheHead();

//    AVRFrame getFIFOHead();
    void getFIFOHead(AVRFrame& frame);

    bool LastFrame2FIFO();

    void getCurrentFrame(AVRFrame & ret);
    void getNextFrame(AVRFrame & ret);
    void getSlamFrame(AVRFrame & ret);
    void getLastFrame(AVRFrame & ret);

    void getCurrentFrame_PointCloud(cv::Mat & ret);

//    long getCurrentFrame_TimeStamp();

    unsigned long long int getLatestZEDTS() ;
    unsigned long long int getLatestFrameTS();
    unsigned long long int getCurrentZEDTS();
    unsigned long long int getCurrentFrameTS();

    void updateCurrFrameFeature();

    void updateLastFrameFeature();

    void cacheExistingFeatureOfAllCacheFrame();

    void getLowPassMotionVectorForCurrFrame();

    void getLowPassFilteredMotionVectorForCurrFrame();
};


#endif //PROJECT_FRAMECACHE_H
