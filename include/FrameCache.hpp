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
    unsigned long long int LatestTS;
    std::vector<AVRFrame> fifo;

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

    void opticalFlowTrack_Curr2Last();

    bool ReachFullMotionBacklog();

    void updatePCDisplacement_Last2FIFOHead();

    void SinkFrames();

    void updateTransformationMatrix_Curr2CacheHead();

    void tranformPointCloud_Curr2CacheHead();

    void updateMotionData_Curr2CacheHead();

    void updateMotionVec_Curr2CacheHead();

    bool LastFrame2FIFO();

    AVRFrame getCurrentFrame();
    AVRFrame getNextFrame();
    AVRFrame getSlamFrame();
    AVRFrame getLastFrame();

    unsigned long long int getLatestTS() ;

    void updateCurrFrameFeature();

    void updateLastFrameFeature();
};


#endif //PROJECT_FRAMECACHE_H
