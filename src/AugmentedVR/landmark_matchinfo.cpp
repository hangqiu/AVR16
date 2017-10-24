//
// Created by nsl on 8/27/16.
//

#include "landmark_matchinfo.hpp"

landmark_matchinfo::landmark_matchinfo(const cv::Mat &lm_src, cv::Mat perspectiveTransformMatrix, int x_dst, int y_dst, int width_dst, int height_dst,
                                       const std::vector<cv::Point2f> &obj, const std::vector<cv::Point2f> &scene)
        : lm_src(lm_src), perspectiveTransformMatrix(perspectiveTransformMatrix), x_dst(x_dst), y_dst(y_dst), width_dst(width_dst), height_dst(height_dst), obj(obj), scene(scene) {}


landmark_matchinfo::landmark_matchinfo(){}

landmark_matchinfo::~landmark_matchinfo() {

}
