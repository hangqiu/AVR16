//
// Created by nsl on 9/20/16.
//

#ifndef PROJECT_GLOBALS_H
#define PROJECT_GLOBALS_H
//ZED Includes
//#include <zed/Mat.hpp>
//#include <zed/Camera.hpp>
//#include <zed/utils/GlobalDefine.hpp>
#include <sl/Camera.hpp>

//opencv includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"


//#define EVAL
//#define SIMPLEEVAL
#define PIPELINE

#define PI 3.14159265

//using namespace cv;
using namespace std;

//extern const int NUM_CAMERAS;
extern const int FPS;
extern int ZEDCACHESIZE;
//extern const sl::zed::ZEDResolution_mode ZED_RES;
//extern const sl::zed::SENSING_MODE senseMode;

// Toggles;
extern bool COOP;
extern bool SHOW_IMG;
extern bool SHOW_CAMMOTION;
extern bool DECOUPLE2IMG;
extern bool SHOW_PC;
extern bool LANDMARK_DISTVECTOR;
extern bool Parallel_TXRX;
extern bool ADAPTIVE_STREAMING;
extern bool OfflineTXRX;
extern bool VehicleControl;
extern int DEBUG;
extern bool LOCKDEBUG;
extern bool VISUAL;
extern bool DEBUGVISUAL;
extern bool SLAMVISUAL;
extern bool PCVISUAL;
extern bool DYNAMICS;
extern int FRAME_ID;
extern int TRACK_FREQ;
extern int DUTYCYCLE;
extern double PCConnectionThresh;
extern double MOTIONTHRESH_PERPIXEL;
extern int PATCHSIZE;// expand step x pixels
extern double HEIGHT_THRESH;
extern bool INIT_FLAG;
extern bool ReuseMap;
//extern bool EVAL;
extern bool PAUSE_FLAG;

extern cv::Size DisplaySize;
extern int MAX_COUNT;

// debug
// TODO: remove
extern int dbx;
extern int dby;
extern int dbw;
extern int dbh;

extern string SEQNO;
extern string FRAME;
extern string TCW;
extern string PC;
extern string TIMESTAMP;
extern string DYNAMICPC;
extern string MOTIONVEC;
extern string LOWPASSMOTIONVEC;

extern string VocFile;
extern string CalibrationFile;

extern const int NUM_CAMERAS;

extern string commPath;
extern string MyAddress;
extern string MyPort;
extern string ServerAddress;
extern string ServerPort;

// store video or webcam
extern bool OFFLINE;

extern bool RX;
extern bool TX;
extern bool SEND;
extern int CamId;
extern int RxCamId;

//int startFrameId = 362;

extern int startFrameId;
extern int lengthInFrame;


extern timeval tInit;

extern bool quit;


void slMat2cvMat(sl::Mat& input, cv::Mat& output);
void cvMat2slMat(cv::Mat& input, sl::Mat& output, sl::MEM);
void stripPointCloudColorChannel(cv::Mat& in, cv::Mat& out);
double timeDifference_msec(timeval& start, timeval& end);
double timeDifference_sec(timeval& start, timeval& end);
unsigned long long getCurrentComputerTimeStamp_usec();

#endif //PROJECT_GLOBALS_H