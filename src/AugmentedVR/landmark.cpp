//
// Created by nsl on 8/24/16.
//

#include <opencv2/core/mat.hpp>
#include "landmark.hpp"

landmark::landmark(const cv::Mat &thumbnail, const cv::Mat &PointCloud, int x, int y, int width, int height) : bbox(thumbnail),
                                                                                                          PointCloud(
                                                                                                                  PointCloud),
                                                                                                          x(x), y(y),
                                                                                                          width(width),
                                                                                                          height(height) {}

landmark::~landmark() {

}
