//
// Created by hang on 9/26/17.
//

#ifndef PROJECT_PCMANIPULATOR_H
#define PROJECT_PCMANIPULATOR_H



void shiftPC(cv::Mat &pc1, cv::Scalar vec);
void onMouseCallback_DisplayDisplacement(int32_t event, int32_t x, int32_t y, int32_t flag, void* param);
cv::Mat MatPerElementNorm(cv::Mat MotionVecMat);
cv::Mat transformPCViaTransformationMatrix_gpu(cv::Mat T, cv::Mat PCReceived);

#endif //PROJECT_PCMANIPULATOR_H
