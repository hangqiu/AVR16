//
// Created by nsl on 9/20/16.
//

#include <opencv2/core/types.hpp>
#include "globals.hpp"
#include <sys/time.h>
#include <string>


using namespace std;


bool COOP = false;
bool TXFRAME_FOREVAL = false;

bool ReuseMap = false;
//bool ReuseMap = true;
bool PIPELINE = false;

bool OfflineTXRX = false;
bool Parallel_TXRX = true;
bool ADAPTIVE_STREAMING = true; ///adapt to the bandwidth
bool VehicleControl = false;

int DEBUG = 0;
bool V2VDEBUG = false;
bool LOCKDEBUG = false;
bool VISUAL = false;
//bool VISUAL = true;
bool DEBUGVISUAL = true;
bool SLAMVISUAL = true;
bool PCVISUAL = true;

bool SHOW_CAMMOTION = true;
//bool SHOW_CAMMOTION = false;

bool DYNAMICS = false;

bool PAUSE_FLAG = false;
//bool PAUSE_FLAG = true;

//const int NUM_CAMERAS = 2;
const int FPS = 15;
int ZEDCACHESIZE = 4;

/// Toggles;
bool SHOW_IMG = true;
bool DECOUPLE2IMG = false;
//bool DECOUPLE2IMG = true;
//bool SHOW_PC = true;

int FRAME_ID = 0;
int TRACK_FREQ = 30;

int MAX_COUNT = 2000;


//int DUTYCYCLE = 1; // x frames per processing

double PCConnectionThresh = 0.02;
double MOTIONTHRESH_PERPIXEL = 0.08; //meters
int PATCHSIZE = 10; // expand step x pixels

double HEIGHT_THRESH = -2;

bool INIT_FLAG = false;
//bool EVAL = false;

bool quit = false;

cv::Size DisplaySize(720, 404);

// debug
// TODO: remove
int dbx = 1000;
int dby = 550;
int dbw = 50;
int dbh = 50;

string SEQNO = "ID";
string FRAME = "Frame";
string TCW = "Tcw";
string PC = "PC";
string TIMESTAMP = "TIMESTAMP";
string DYNAMICPC = "DYNAMICPC";
string MOTIONVEC = "MOTIONVEC";
string LOWPASSMOTIONVEC = "LOWPASSMOTIONVEC";

string VocFile = "./src/ORB_SLAM2/Vocabulary/ORBvoc.bin";
string CalibrationFile = "./CamCalib.yaml";

const int NUM_CAMERAS = 1;

/// Comm
string commPath = "/home/hang/AVRComm/";
////string AddressA = "http://204.57.7.36:";
string AddressA = "192.168.0.166";
string PortA = "34568";
string AddressB = "192.168.0.187";
string PortB = "34569";

/// laptop 187
string MyAddress = AddressB;
string MyPort = PortB;
string ServerAddress = AddressA;
string ServerPort = PortA;
/// desktop 166
//string MyAddress = AddressA;
//string MyPort = PortA;
//string ServerAddress = AddressB;
//string ServerPort = PortB;
/// store video or webcam
bool OFFLINE = true;

//// single solo
bool RX = false;
bool TX = false;
bool SEND = false;
int CamId = 0;
int RxCamId = 1;


//// Receiver
//bool RX = true;
//bool TX = false;
//bool SEND = false;
//int CamId = 1;
//int RxCamId = 0;

//// Sender
//bool RX = false;
//bool TX = true;
//bool SEND = true;
//int CamId = 0;
//int RxCamId = 1;

int startFrameId = 0;
int lengthInFrame = 999999999;

timeval tInit;


void slMat2cvMat(sl::Mat& input, cv::Mat& output) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU)).copyTo(output);
}


void cvMat2slMat(cv::Mat& input, sl::Mat& output, sl::MEM memType) {
    // Mapping between MAT_TYPE and CV_TYPE
    sl::MAT_TYPE sl_type;
    switch (input.type()) {
        case CV_32FC1: sl_type = sl::MAT_TYPE_32F_C1; break;
        case CV_32FC2: sl_type = sl::MAT_TYPE_32F_C2; break;
        case CV_32FC3: sl_type = sl::MAT_TYPE_32F_C3; break;
        case CV_32FC4: sl_type = sl::MAT_TYPE_32F_C4; break;
        case CV_8UC1: sl_type = sl::MAT_TYPE_8U_C1; break;
        case CV_8UC2: sl_type = sl::MAT_TYPE_8U_C2; break;
        case CV_8UC3: sl_type = sl::MAT_TYPE_8U_C3; break;
        case CV_8UC4: sl_type = sl::MAT_TYPE_8U_C4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    sl::uchar1* ptr = (sl::uchar1*)(void*)(input.data);
    size_t step = (size_t)(input.step);
    size_t width = (size_t)(input.size().width);
    size_t height = (size_t)(input.size().height);
    output = sl::Mat(width, height, sl_type, ptr, step, memType);
    return;
}


void stripPointCloudColorChannel(cv::Mat& in, cv::Mat& out){
    assert(in.channels()==4);
    cv::Mat ret;
    cv::Mat PCChannels[3];

    for (int i=0;i<3;i++){
        cv::extractChannel(in,PCChannels[i],i);
    }
    merge(PCChannels,3,out);
}

double timeDifference_msec(timeval& startT, timeval& endT){
    return double(endT.tv_sec-startT.tv_sec)*1000 + double(endT.tv_usec-startT.tv_usec) / 1000;
}

double timeDifference_sec(timeval& startT, timeval& endT){
    return (double(endT.tv_sec-startT.tv_sec)*1000 + double(endT.tv_usec-startT.tv_usec) / 1000) / 1000;
}

unsigned long long getCurrentComputerTimeStamp_usec(){
    timeval ts;
    gettimeofday(&ts,NULL);
    unsigned long long ret = (ts.tv_sec*1000000 + ts.tv_usec);
    return ret;
}