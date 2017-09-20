/*
 * CMT_vec.h
 *
 *  Created on: Nov 30, 2015
 *      Author: nsl
 */

#ifndef CMT_VEC_H_
#define CMT_VEC_H_


#include "src/DeepCam/CMT/common.h"
#include "src/DeepCam/CMT/Consensus.h"
#include "src/DeepCam/CMT/Fusion.h"
#include "src/DeepCam/CMT/Matcher.h"
#include "src/DeepCam/CMT/Tracker.h"
#include "src/DeepCam/CMT/CMT.h"

#include <opencv2/features2d/features2d.hpp>

using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::RotatedRect;
using cv::Size2f;
using namespace cmt;


class CMT_VEC
{
public:
	CMT_VEC() : str_detector("FAST"), str_descriptor("BRISK") {};
    void initialize(const Mat im_gray, const Rect rect,const Mat im_color);
    void processFrame(const Mat im_gray);

    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;
//
    string str_detector;
    string str_descriptor;
//
    vector<Point2f> points_active; //public for visualization purposes
    RotatedRect bb_rot;

private:
    vector<CMT>  cmt_vec;
    Ptr<cv::BRISK> detector;
    Ptr<DescriptorExtractor> descriptor;

    Size2f size_initial;
//
    vector<int> classes_active;

    float theta;

    Mat im_prev;
};


#endif /* CMT_VEC_H_ */
