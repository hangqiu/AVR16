//
// Created by nsl on 9/17/16.
// This class track existing_obj to moved_obj by calculating optical flows
//

#ifndef MOBILEVIDEONET_CLEAN_MULTIOBJTRACKER_H
#define MOBILEVIDEONET_CLEAN_MULTIOBJTRACKER_H


#include "globals.h"
#include "Object.h"


struct LKT_IDX{
    std::vector<int> idx_vec;
};

class Cam;
class MultiObjTracker {
private:
//    cv::Mat prevGray;
//    cv::Mat curGray;
//    std::vector<Object> existing_obj;
//    std::vector<Object> moved_obj;


    Cam* camera;
//keypoints from prev frame and cur frame

    std::vector<cv::Point2f> points[2];
    std::vector<cv::Point2f> points_back;
    std::vector<uchar> status;

    std::vector<float> err;
    std::vector<unsigned char> status_back;
    std::vector<float> err_back; //Needs to be float

    std::vector<struct LKT_IDX> lkt_obj;

// tracked objs.

    Point2f totalOpticalFlow;
    Point2f frameTotalOpticalFlow;

public:
    MultiObjTracker(Cam *camera);

    virtual ~MultiObjTracker();

    const Point2f &getFrame_total_op_flow() const;

    const vector<Point2f> *getPoints() const;
    const vector<uchar> &getStatus() const;

    std::vector<cv::Point2f> detect_keypoints(cv::Mat im);


    const vector<LKT_IDX> &getLkt_obj() const;

    void TrackMultiObj();

};


#endif //MOBILEVIDEONET_CLEAN_MULTIOBJTRACKER_H
