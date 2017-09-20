//
// Created by nsl on 9/17/16.
//

#ifndef MOBILEVIDEONET_CLEAN_DISPLAYER_H
#define MOBILEVIDEONET_CLEAN_DISPLAYER_H

#include "globals.h"
#include "Object.h"
#include "DeepCam.h"

class Displayer {

private:
    Cam* camera;

//    Mat im_0, im_0_gray; // original frame
//    Mat curIm, curGray; // resized frame
//    Mat prevIm, prevGray;
//    // TODO: draw all things on prevIm fow now. messy
//    std::vector<Object> existing_obj;

    MultiObjTracker* multiObjTracker;

//    std::vector<cv::Point2f> points[2];//keypoints from prev frame and cur frame
//    std::vector<Point2f> points_back;
//    std::vector<uchar> status;
//    std::vector<float> err;
//    std::vector<unsigned char> status_back;
//    std::vector<float> err_back; //Needs to be float

public:
    Displayer(Cam *camera, MultiObjTracker *multiObjTracker);

    virtual ~Displayer();

    int displayExistingObjOnPrevIm(int R, int G, int B);

    void drawKPTMatchesOnPrevIm();// TODO: draw on curGray now
    void showOriginalIm(string basic_string);

    void showCurIm(string basic_string);
};


#endif //MOBILEVIDEONET_CLEAN_DISPLAYER_H
