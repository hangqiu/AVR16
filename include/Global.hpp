//
// Created by hang on 9/25/17.
//

#ifndef PROJECT_GLOBAL_H
#define PROJECT_GLOBAL_H

//ZED Includes
#include <sl/Camera.hpp>

//opencv includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


//#define EVAL
//#define SIMPLEEVAL
//#define PIPELINE

#define PI 3.14159265


using namespace std;

class Global {
public:

// Config:
// store video or webcam
    bool OFFLINE;

    bool RX;
    bool TX;
    bool SEND;
    int CamId;
    int RxCamId;


    int startFrameId;
    int lengthInFrame;

     const int FPS;
     int ZEDCACHESIZE;

    // Variables
    bool quit = false;

// Toggles;
     bool COOP;
     bool SHOW_IMG;
     bool SHOW_CAMMOTION;
     bool DECOUPLE2IMG;
     bool SHOW_PC;
     bool LANDMARK_DISTVECTOR;
     int DEBUG;
     bool VISUAL;
     bool DEBUGVISUAL;
     bool SLAMVISUAL;
     int FRAME_ID;
     int TRACK_FREQ;
     int DUTYCYCLE;
     double PCConnectionThresh;
     double MOTIONTHRESH_PERPIXEL;
     int PATCHSIZE;// expand step x pixels
     double HEIGHT_THRESH;
     bool INIT_FLAG;
     bool ReuseMap;
// bool EVAL;
     bool PAUSE_FLAG;

     cv::Size DisplaySize;

// debug
// TODO: remove
     int dbx;
     int dby;
     int dbw;
     int dbh;

     string FRAME;
     string TCW;
     string PC;
     string TIMESTAMP;
     string DYNAMICPC;
     string MOTIONVEC;

     string VocFile;
     string CalibrationFile;

     const int NUM_CAMERAS;
     string commPath;

     timeval tInit;




    cv::Mat slMat2cvMat(sl::Mat& input);
    void cvMat2slMat(cv::Mat& input, sl::Mat& output);
};


#endif //PROJECT_GLOBAL_H
