//
// Created by nsl on 9/20/16.
//

#ifndef PROJECT_DISPLAYER_H
#define PROJECT_DISPLAYER_H


#include "AugmentedVR.hpp"
//#include "PointCloud.hpp"
//#include "Viewer.hpp"
#include "GLViewer.hpp"
#include <include/globals.hpp>

#include "opencv2/highgui/highgui.hpp"


class Displayer {

public:
    AugmentedVR** VNode;
//    PointCloud* mPC;
//    Viewer* mPCViewer;

// SDK 2.0 Point cloud viewer
    GLViewer* mGLViewer;
    std::thread display_callback;

//    sl::Mat PC_gpu;

    int PCwidth, PCheight; //
//    CUcontext context;
public:
    Displayer(AugmentedVR **VNode);

    ~Displayer();

    void exit();

    void saveFrame();

    void processKey(char key);

    void pauseStopResume();

    void init(int argc, char **argv);

    void showDynamicPC();

    void showMergedPC(cv::Mat mat);

    void showPC(cv::Mat mat);

    void showPC(sl::Mat& mat);

    void checkResetPCViewer(int width, int height);

    void pushPC_cvMat(cv::Mat &mat);

//    void debugPC(cv::Mat DebugPC);

    void showPC();

    void showSplitScreen(cv::Mat PC1, cv::Mat PC2);

    char showCurFrame();

    void showCurFrameWithPC();

    void showCurDynamicFrame(int idx);


    void showImgWithPC(cv::Mat& img, cv::Mat* PC, char * winName);

//    void onMouseCallback_DisplayVoxel(int32_t event, int32_t x, int32_t y, int32_t flag, void* param);
};


#endif //PROJECT_DISPLAYER_H
