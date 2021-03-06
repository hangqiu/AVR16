//
// Created by hang on 9/26/17.
//

#ifndef PROJECT_CVUTILS_H
#define PROJECT_CVUTILS_H

#include <opencv2/core/types.hpp>
#include <include/globals.hpp>




void debugCin();
void debugPauser();
void processKey(char key);
void pauseStopResume();
void saveCurFrame(cv::Mat FrameLeft, int frameSeq, long frameTS);
void saveOpenGL(int width, int height);
void detectKLTFeature(cv::Mat & FrameLeftGray, vector<cv::Point2f> & keypoints);
void drawMatchedKeypoints(cv::Mat & img, cv::Point2f& kp1, cv::Point2f& kp2, string txtAtKp1);
void onMouseCallback_DisplayVoxel(int32_t event, int32_t x, int32_t y, int32_t flag, void* param);

#endif //PROJECT_CVUTILS_H
