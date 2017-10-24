//
// Created by nsl on 8/24/16.
//

#ifndef PROJECT_LANDMARK_H
#define PROJECT_LANDMARK_H

class landmark{
public:
    landmark(const cv::Mat &thumbnail, const cv::Mat &PointCloud, int x, int y, int width, int height);

    virtual ~landmark();

public:
    cv::Mat bbox;
    cv::Mat PointCloud;
    int x,y,width,height;
};

#endif //PROJECT_LANDMARK_H
