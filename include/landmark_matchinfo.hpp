//
// Created by nsl on 8/27/16.
//

#ifndef PROJECT_LANDMARK_MATCHINFO_H
#define PROJECT_LANDMARK_MATCHINFO_H

//#include "landmark.h"
#include <vector>
#include "opencv2/core/core.hpp"

class landmark_matchinfo{
public:
//    landmark lm_src;
    cv::Mat lm_src;
    cv::Mat perspectiveTransformMatrix;
    int x_dst,y_dst,width_dst,height_dst;
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    landmark_matchinfo();
    landmark_matchinfo(const cv::Mat &lm_src, cv::Mat perspectiveTransformMatrix, int x_dst, int y_dst, int width_dst, int height_dst,
                       const std::vector<cv::Point2f> &obj, const std::vector<cv::Point2f> &scene);

    virtual ~landmark_matchinfo();
};


#endif //PROJECT_LANDMARK_MATCHINFO_H
