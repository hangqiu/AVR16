//
// Created by nsl on 9/16/16.
// This is the toppest level camera processing class
//

#ifndef MOBILEVIDEONET_CLEAN_CAM_H
#define MOBILEVIDEONET_CLEAN_CAM_H

#include <iostream>
#include "YOLO.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "MultiObjTracker.h"
#include "Displayer.h"
#include "IO.h"
#include "MultiObjMatcher.h"
#include "globals.h"

using namespace std;
using namespace cv;

class IO;
class MultiObjTracker;
class MultiObjMatcher;
class Displayer;
class YOLO;

class DeepCam {

private:

    cv::VideoCapture cap;
    Mat im_0, im_0_gray; // original frame
    Mat curIm, curGray;
    Mat prevIm, prevGray;

    int rows, cols, resizeRows, resizeCols;


    // Cam info
    string id;
    int fps;
    double azimuth;
    double lat, lon;

    // frame info
    int curFrameId;
    long curTS;

    // Obj Info
    double scale;


//    these are parallel to cam
//    Displayer* displayer;
//    IO* io;


public:

    // Members
    YOLO* yolo;
    MultiObjTracker* multiObjTracker;
    MultiObjMatcher* multiObjMatcher;

    vector<Object> existing_obj;
    vector<Object> moved_obj;
    vector<Object> new_obj;

public:

    Cam();

    Cam(string input_path, string& DatasetPath, string& CamID,string& metadata_path, string& cfgfile, string& weightfile, IO* io);

    virtual ~Cam();

    vector<Object> &getExisting_obj();

    vector<Object> &getMoved_obj();

    vector<Object> &getNew_obj();

    void setId(char *id);

    void setFps(int fps);

    void setAzimuth(double azimuth);

    void setLat(double lat);

    void setLon(double lon);

    void setCurTS(long curTS);

    double getScale() const;

    int getRows() const;

    int getCols() const;

    int getResizeRows() const;

    int getResizeCols() const;

    char *getCamID() const;

    int getCurFrameId() const;

    int getFps() const;

    double getAzimuth() const;

    double getLat() const;

    double getLon() const;

    long getCurTS() const;

    const Mat &getIm_0() const;

    const Mat &getIm_0_gray() const;

    const Mat &getCurIm() const;

    const Mat &getCurGray() const;

    const Mat &getPrevIm() const;

    const Mat &getPrevGray() const;




    void setMoved_obj(const vector<Object> &moved_obj);

    void pushBackExisting_obj(Object object);

//    Cam(){}; // default constructor
//    Cam(char* CamID): WIN_NAME("Gotcha"),SHOW_IMG(true),id(CamID){}; // TODO: move camid to init
//    ~Cam(){};


    bool loadFrame();

    void MultiObjMatching();

    Mat resize_Input_gray(Mat im_in, double scale);
    Mat get_gray(Mat im0);
    void pause_step_resume();

    bool process_key(char key);

    Mat resize_Input_color(Mat im_in, double scale);

    void initYOLOonInitialFrame(){
        existing_obj = yolo->run_yolo(curIm, (char*)id.c_str());
    }
    void runYOLOonNewFrame(){
//        return yolo.run_yolo(im0,id);
        cout << "YOLO ... frame " << curFrameId << endl;
        new_obj = yolo->run_yolo(curIm, (char*)id.c_str());
    }


    bool processTrigger();

    void deadReckoning();

    void evictObj();
};


#endif //MOBILEVIDEONET_CLEAN_CAM_H
