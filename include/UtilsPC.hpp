//
// Created by hang on 9/26/17.
//

#ifndef PROJECT_PCMANIPULATOR_H
#define PROJECT_PCMANIPULATOR_H

struct xyzNorm{
    float x,y,z;
    double norm;
};

void shiftPC(cv::Mat &pc1, cv::Scalar vec);
void onMouseCallback_DisplayDisplacement(int32_t event, int32_t x, int32_t y, int32_t flag, void* param);
cv::Mat MatPerElementNorm(cv::Mat MotionVecMat);
void transformPC_Via_TransformationMatrix(cv::Mat& T, cv::Mat& PCReceived, cv::Mat &ret);
void transformPC_Via_TransformationMatrix(cv::Mat& tlc, cv::Mat& Rlc, cv::Mat& PCReceived, cv::Mat &ret);
void debugPC(cv::Mat DebugPC);

#endif //PROJECT_PCMANIPULATOR_H
