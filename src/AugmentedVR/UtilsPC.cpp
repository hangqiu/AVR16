//
// Created by hang on 9/26/17.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <include/globals.hpp>
#include <queue>
#include "UtilsPC.hpp"
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


//cv::Mat transformPCViaTransformationMatrix_gpu(cv::Mat T, cv::Mat PCReceived){
//    // T is 4 by 4 transformation matix, where 3 by 3 rot, and 1 by 3 translation, last row is 0,0,0,1
//    // point cloud is 1 by 4 (x,y,z,rgba)
//    assert(T.size().width==4 && T.size().height==4);
//    cv::Mat ret;
//    cout << T << endl;
//    //rotation
//    for (int i=0;i<3;i++){
//        cv::Scalar scalar_ = cv::Scalar(T.at<float>(i,0),T.at<float>(i,1),T.at<float>(i,2),T.at<float>(i,3));
//        cout << scalar_ << endl;
//        cv::Mat res;
//        cv::cuda::multiply(PCReceived, scalar_, res);
//        ret += res;
//    }
//    //translation
//    cv::Mat res;
//    cv::Scalar scalar_ = cv::Scalar(T.at<float>(3,0),-T.at<float>(3,1),-T.at<float>(3,2),T.at<float>(3,3));
//    cout << scalar_ << endl;
//    cv::cuda::multiply(PCReceived, scalar_, res);
//    ret += res;
//
////    cv::cuda::multiply(PCReceived,T,ret);
//    return ret;
//}

void transformPC_Via_TransformationMatrix(cv::Mat& Trc, cv::Mat& PCReceived, cv::Mat &ret){
    // T is 4 by 4 transformation matix, where 3 by 3 rot, and 1 by 3 translation, last row is 0,0,0,1
    // point cloud is 1 by 4 (x,y,z,rgba)
    assert(Trc.size().width==4 && Trc.size().height==4);
    // extract channels
    cv::Mat PCChannels[4];
//    cv::Mat ret;
    for (int i=0;i<4;i++){
        cv::extractChannel(PCReceived,PCChannels[i],i);
    }

    //watchout coord sys diff
//    PCChannels[0] = -PCChannels[0];
//    PCChannels[1] = -PCChannels[1];
//    PCChannels[2] = -PCChannels[2];

//    cout << Trc << endl;

    cv::Mat interRes[4][4];
    for (int i=0;i<3;i++){
        // for each channel
        for (int j=0;j<3;j++){
            // watchout: T.at(row,col)
//            cv::cuda::multiply(PCChannels[i], cv::Scalar(T.at<float>(i,j)), interRes[i][j]);
//            cout << Trc.at<float>(j,i) << endl;
            interRes[i][j] = PCChannels[i] * Trc.at<float>(j,i);
        }
    }

    PCChannels[0] = interRes[0][0] + interRes[1][0] + interRes[2][0] + Trc.at<float>(0,3);
    PCChannels[1] = interRes[0][1] + interRes[1][1] + interRes[2][1] + Trc.at<float>(1,3);
    PCChannels[2] = interRes[0][2] + interRes[1][2] + interRes[2][2] + Trc.at<float>(2,3);
    //set coord sys back
//    PCChannels[0] = -PCChannels[0];
//    PCChannels[1] = -PCChannels[1];
//    PCChannels[2] = -PCChannels[2];

    merge(PCChannels,4,ret);
//    return ret;
}

void transformPC_Via_TransformationMatrix(cv::Mat& tlc, cv::Mat& Rlc, cv::Mat& PCReceived, cv::Mat &ret){
    cv::Mat Tlc = cv::Mat::eye(4,4,CV_32F);
    tlc.copyTo(Tlc.rowRange(0,3).col(3));
    Rlc.copyTo(Tlc.rowRange(0,3).colRange(0,3));
    transformPC_Via_TransformationMatrix(Tlc,PCReceived,ret);
}

void debugPC(cv::Mat DebugPC){
    if (!DEBUG) return;
//    cv::Mat DebugPC = VNode[0]->DynamicPC;
    cout 	<< "PC dims:" << DebugPC.rows
            << ", "<< DebugPC.cols
            << ", "<< DebugPC.channels() <<  endl;
    cout    << "PointCloud element size: " << DebugPC.elemSize()
            << " Byte, total size: " << DebugPC.elemSize() * DebugPC.rows * DebugPC.cols << endl;
    cv::Mat DebugPCChan;
    cv::extractChannel(DebugPC,DebugPCChan,0);
//                cv::Mat mask = cv::Mat(DebugPCChan!=DebugPCChan);
    cout    << "NonZero elements: " << cv::countNonZero(DebugPCChan) << endl; // can only count single channel matrix
//                cout << cv::countNonZero(mask);
    cout << "Value in the middle, " << DebugPCChan.at<float>(DebugPC.cols/2,DebugPC.rows/2) << endl;
    cout << "Value in the middle, " << DebugPC.at<cv::Vec4f>(DebugPC.cols/2,DebugPC.rows/2) << endl;
}


void removePointCloud_HighLow(cv::Mat &PCIn, cv::Mat & PCOut){
    cv::Mat tmpMask,tmpPC;
    /// extract y channel
    extractChannel(PCIn,tmpPC,1);

    cv::threshold( tmpPC, tmpMask, HEIGHT_THRESH, 255, cv::THRESH_BINARY_INV);
    tmpMask.convertTo(tmpMask,CV_8U);
    PCIn.copyTo(PCOut);
}