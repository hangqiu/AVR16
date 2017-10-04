//
// Created by hang on 9/26/17.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <include/globals.hpp>
#include <queue>
#include "PCUtils.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"


void shiftPC(cv::Mat& pc1,cv::Scalar vec){
    cv::Mat shift(pc1.rows, pc1.cols, CV_32FC4, vec);
    pc1=shift+pc1;
}

cv::Mat MatPerElementNorm(cv::Mat MotionVecMat){

    cv::Mat PCChannels[3];
    cv::Mat PCChan2[3];
    cv::Mat PCDisplacement(MotionVecMat.size().height,MotionVecMat.size().width, CV_32F, cv::Scalar(0.));

    for (int i=0;i<3;i++){
        cv::extractChannel(MotionVecMat,PCChannels[i],i);
        if (DEBUG>1)cout << "channel" << i << ": "<< PCChannels[i] << endl;
        PCChan2[i] = PCChannels[i].mul(PCChannels[i]);
        PCDisplacement += PCChan2[i];
        if (DEBUG>1)cout << "PCDisplacement square added: " <<PCDisplacement << endl;
    }


    if (DEBUG>1) cout << "PCDisplacement square: " <<PCDisplacement << endl;
    cv::sqrt(PCDisplacement,PCDisplacement);
    if (DEBUG>1) cout << "PCDisplacement: " <<PCDisplacement << endl;
    // TODO: reenable
//    if (DEBUG>1) myfile << PCDisplacement << endl;
    return PCDisplacement;
}


cv::Mat transformPCViaTransformationMatrix_gpu(cv::Mat T, cv::Mat PCReceived){
    // T is 4 by 4 transformation matix, where 3 by 3 rot, and 1 by 3 translation, last row is 0,0,0,1
    // point cloud is 1 by 4 (x,y,z,rgba)
    cv::Mat ret;
//    cv::cuda::GpuMat T_gpu(T);
//    cv::cuda::GpuMat PCReceived_gpu(PCReceived);
    cv::cuda::multiply(PCReceived, T, ret);
//    ret  = cv::Mat(T_gpu);
    return ret;
}