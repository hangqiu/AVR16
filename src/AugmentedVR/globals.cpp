//
// Created by nsl on 9/20/16.
//

#include <opencv2/core/types.hpp>
#include "globals.hpp"
#include <string>



using namespace std;


bool COOP = true;

bool ReuseMap = false;
//bool ReuseMap = true;

int DEBUG = 0;
//bool VISUAL = false;
bool VISUAL = true;
bool DEBUGVISUAL = true;
bool SLAMVISUAL = true;

//bool PAUSE_FLAG = false;
bool PAUSE_FLAG = true;

//const int NUM_CAMERAS = 2;
const int FPS = 15;
int ZEDCACHESIZE = 6;

// Toggles;
bool SHOW_IMG = true;
bool DECOUPLE2IMG = false;
//bool DECOUPLE2IMG = true;
bool SHOW_PC = true;
bool LANDMARK_DISTVECTOR = false;

int FRAME_ID = 0;
int TRACK_FREQ = 30;

bool SHOW_CAMMOTION = false;
int DUTYCYCLE = 5; // x frames per processing

double PCConnectionThresh = 0.02;
double MOTIONTHRESH_PERPIXEL = 0.05; //meters
int PATCHSIZE = 10; // expand step x pixels

double HEIGHT_THRESH = 2.5;

bool INIT_FLAG = false;
//bool EVAL = false;

cv::Size DisplaySize(720, 404);

// debug
// TODO: remove
int dbx = 1000;
int dby = 550;
int dbw = 50;
int dbh = 50;

string FRAME = "Frame";
string TCW = "Tcw";
string PC = "PC";
string TIMESTAMP = "TIMESTAMP";
string DYNAMICPC = "DYNAMICPC";
string MOTIONVEC = "MOTIONVEC";

string VocFile = "./src/ORB_SLAM2/Vocabulary/ORBvoc.txt";
string CalibrationFile = "./CamCalib.yaml";

const int NUM_CAMERAS = 1;

const sl::zed::ZEDResolution_mode ZED_RES = sl::zed::ZEDResolution_mode::HD720;
const sl::zed::SENSING_MODE senseMode = sl::zed::SENSING_MODE::FILL;
string commPath = "/home/nsl/comm/";
// store video or webcam
bool OFFLINE = true;

//
//bool RX = true;
//bool TX = false;
//bool SEND = false;
//int CamId=0;
//int RxCamId = 1;

//int startFrameId = 300; // receriver 200 frame can be relocailized
//int startFrameId = 0;

bool RX = false;
bool TX = true;
bool SEND = false;
//bool SEND = true;
int CamId=1;
int RxCamId = 0;

//int startFrameId = 362;

int startFrameId = 1;
int lengthInFrame = 100;

timeval tInit;
