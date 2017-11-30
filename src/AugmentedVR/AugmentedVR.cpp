//
// Created by nsl on 9/20/16.
//



#include "AugmentedVR.hpp"
#include <ctime>
#include <sys/time.h>
#include <include/UtilsPC.hpp>
#include <include/UtilsCV.hpp>

using namespace sl;

//#define EVAL


AugmentedVR::AugmentedVR(int CamId, sl::InitParameters initParam, sl::RuntimeParameters runtimeParam, int ZEDConfidence) {
    ZED_LRes = cv::Mat(DisplaySize, CV_8UC4);
//
//    AugmentedVR::senseMode = senseMode;
    mZEDCam = new sl::Camera();
    initParameters = initParam;
    runtimeParameters = runtimeParam;
    // remove the not to be trusted data
    AugmentedVR::ZEDConfidence = ZEDConfidence;
    AugmentedVR::CamId = CamId;
    startTS = 0;
    initialized = false;
    mIo = new IO(this);
}

AugmentedVR::~AugmentedVR(){

    delete(mZEDCam);
    delete(mSLAM);
}

void AugmentedVR::exit(){
    if (!quit)mSLAM->SaveTrajectoryKITTI("CameraTrajectory.txt");
    mSLAM->Shutdown();

//    pointcloud_sl_gpu.free(MEM_GPU);

    mZEDCam->close();
    cout << "AVR shuts down" << endl;
}

int AugmentedVR::initZEDCam(int startFrameID){

    sl::ERROR_CODE err = mZEDCam->open(initParameters);
    cout << "ZED N°" << CamId << " -> Result : " << errorCode2str(err).c_str() << endl;
    if (err != sl::SUCCESS) {
        delete mZEDCam;
        std::exit(-1);
    }
    Resolution image_size = mZEDCam->getResolution();
    width = (int)image_size.width;
    height = (int)image_size.height;
//    mZEDCam->setConfidenceThreshold(ZEDConfidence);

    /// get initial time stamp
    grabNextZEDFrameOffline();


    int tmp_id = 0;
    while (tmp_id++ < startFrameID){
        cerr << "skipping frame " << tmp_id << endl;
        grabNextZEDFrameOffline();
    }

    /// Here starts the frame seq 0.
    startTS = frameCache.getLatestZEDTS();
    /// re-caliberate the time to absolute timestamp
    ZEDStartTSOffset = getCurrentComputerTimeStamp_usec()*1000 - mZEDCam->getCurrentTimestamp();

    frameCache.setStartTS(startTS);
    TotalFrameSeq = 0;
    grabNextZEDFrameOffline();

    if (!frameCache.NextFrame2SlamFrame()){
        if (DEBUG) cerr<< " AugmentedVR::initZEDCam: SlamFrame2CurrentFrame Failure\n";
    };
    grabNextZEDFrameOffline();
    return 0;
}


void AugmentedVR::initSLAMStereo(string VocFile, string CalibrationFile, bool bReuseMap, string mapFile){

    if (bReuseMap == true){
        mSLAM = new ORB_SLAM2::System(VocFile,CalibrationFile,ORB_SLAM2::System::STEREO,true, bReuseMap, mapFile);
    }else{
        mSLAM = new ORB_SLAM2::System(VocFile,CalibrationFile,ORB_SLAM2::System::STEREO,true, bReuseMap);
    }
    FeedSlamTheSlamFrame();
}

bool AugmentedVR::grabNextZEDFrameOffline() {

    if  (mZEDCam->grab(runtimeParameters) != SUCCESS){
        cerr << "AugmentedVR::grabNextZEDFrameOffline: can't grab image" << endl;
        return false;
    }

    frameCache.LoadNextFrameFromZED(mZEDCam, width, height, TotalFrameSeq++);

    if (!INIT_FLAG) {
        AVRFrame next;
        frameCache.getNextFrame(next);
        next.pointcloud.copyTo(initPC);//TODO
        INIT_FLAG = true;
//        imshow("retrieve new", next.FrameLeft);
    }

    return true;
}

void AugmentedVR::FeedSlamTheSlamFrame(){
#ifdef EVAL
    timeval start,end;
    gettimeofday(&start, NULL);
#endif
    AVRFrame slam;
    frameCache.getSlamFrame(slam);
    mSLAM->mTracker_CreateNextFrame(slam.FrameLeftGray,
                                    slam.FrameRightGray,
                                    slam.frameTS);
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "FeedSlamNextFrame >>>>>>> Load Next Frame: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
#endif
}


void AugmentedVR::getCurrentAVRFrame(AVRFrame &ret){
    frameCache.getCurrentFrame(ret);
}

void AugmentedVR::getLastAVRFrame(AVRFrame &ret){
    frameCache.getLastFrame(ret);
}

void AugmentedVR::getCurrentAVRFrame_PointCloud(cv::Mat &ret){
    frameCache.getCurrentFrame_PointCloud(ret);
}

unsigned long long int AugmentedVR::getCurrentAVRFrame_TimeStamp_FrameTS(){
    return frameCache.getCurrentFrameTS();
}

unsigned long long int AugmentedVR::getCurrentAVRFrame_TimeStamp_ZEDTS(){
    return frameCache.getCurrentZEDTS();
}

unsigned long long int AugmentedVR::getCurrentAVRFrame_AbsoluteTimeStamp(){
    return frameCache.getCurrentZEDTS() + ZEDStartTSOffset;
}

void AugmentedVR::SinkFrames(){
    frameCache.SinkFrames();
    mSLAM->mTracker_LoadNextFrameAsCurrent();
}

//bool AugmentedVR::PrepareNextFrame() {
//
//
//#ifdef PIPELINE
//    thread prefetch(&AugmentedVR::grabNextZEDFrameOffline,this);
//    thread feedslam(&AugmentedVR::FeedSlamTheSlamFrame,this);
//#else
//    if (!grabNextZEDFrameOffline()) return false;
//    FeedSlamTheSlamFrame();
//#endif
//
//#ifdef PIPELINE
//    prefetch.join();
//    feedslam.join();
//#endif
//    return true;
//}

void AugmentedVR::calcOpticalFlow(){
    if (!initialized){
        frameCache.updateCurrFrameFeature();
        initialized = true;
        return;
    }
//    if (FRAME_ID%DUTYCYCLE==0){
    if (TotalFrameSeq%ZEDCACHESIZE==0){
        frameCache.updateLastFrameFeature();
        frameCache.cacheExistingFeatureOfAllCacheFrame();
    }
    frameCache.opticalFlowTrack_Curr2Last();
}



// call SLAM and calculate camera pose relative to last frame
// update rotation and translation matrxi to last frame: Rlc, tlc
void AugmentedVR::trackCam() {
    AVRFrame currFrame;
    getCurrentAVRFrame(currFrame);
    (mSLAM->TrackStereo(currFrame.FrameLeft,
                        currFrame.FrameRight,
                        currFrame.frameTS))
            .copyTo(frameCache.CurrentFrame.CamMotionMat); //TODO: conflict with FeedSlamNextFrame? No
    // for debug only
//    if(COOP){
//        CamMotionMat.at<float>(0,3) +=7;
//    }
    if (SHOW_CAMMOTION) cout << "CamMotionMat: \n" << frameCache.CurrentFrame.CamMotionMat << endl;
}

bool AugmentedVR::trackGood(){
    AVRFrame currFrame;
    getCurrentAVRFrame(currFrame);
    return !(currFrame.CamMotionMat.empty());
}

void AugmentedVR::analyze(){
#ifdef EVAL
    timeval tTotalStart, tFetchStart, tCacheStart, tSlamStart, tPCMotionStart, tPCMotionFilterStart, tObjectFilterStart, tTXStart, tRXStart, tPCmergeStart,tDeadReckonStart;
    timeval tTotalEnd, tFetchEnd, tCacheEnd, tSlamEnd, tPCMotionEnd, tPCMotionFilterEnd, tObjectFilterEnd, tTXEnd, tRXEnd, tPCmergeEnd, tDeadReckonEnd;
#endif
    if (!frameCache.FullBacklogAfterSLAM()){
        cerr << "AugmentedVR::analyze(): not enough history frames\n";
        return;
    }
#ifdef EVAL
    gettimeofday(&tPCMotionStart, NULL);
#endif
    PCMotionAnalysis();
#ifdef EVAL
    gettimeofday(&tPCMotionEnd, NULL);
    cout << "TimeStamp: " << double(tPCMotionEnd.tv_sec-tInit.tv_sec)*1000 + double(tPCMotionEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "analyze>>>>>>PC Motion: " << double(tPCMotionEnd.tv_sec-tPCMotionStart.tv_sec)*1000 + double(tPCMotionEnd.tv_usec-tPCMotionStart.tv_usec) / 1000<< "ms" << endl;
    gettimeofday(&tPCMotionFilterStart, NULL);
#endif
#ifdef EVAL
    gettimeofday(&tPCMotionFilterEnd, NULL);
    cout << "TimeStamp: " << double(tPCMotionFilterEnd.tv_sec-tInit.tv_sec)*1000 + double(tPCMotionFilterEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "analyze>>>>>>PC Motion Filter: " << double(tPCMotionFilterEnd.tv_sec-tPCMotionFilterStart.tv_sec)*1000 + double(tPCMotionFilterEnd.tv_usec-tPCMotionFilterStart.tv_usec) / 1000<< "ms" << endl;
    gettimeofday(&tObjectFilterStart, NULL);
#endif
    ObjectMotionAnalysis();
#ifdef EVAL
    gettimeofday(&tObjectFilterEnd, NULL);
    cout << "TimeStamp: " << double(tObjectFilterEnd.tv_sec-tInit.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "analyze>>>>>>PC Object Filter: " << double(tObjectFilterEnd.tv_sec-tObjectFilterStart.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tObjectFilterStart.tv_usec) / 1000<< "ms"<< endl;
    cout << "TimeStamp: " << double(tObjectFilterEnd.tv_sec-tInit.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "analyze: " << double(tObjectFilterEnd.tv_sec-tPCMotionStart.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tPCMotionStart.tv_usec) / 1000<< "ms"<< endl;
#endif
}

void AugmentedVR::PCMotionAnalysis() {
    // choose the oldest and cacl the displacement
    assert(frameCache.FullBacklogAfterSLAM());
    frameCache.updateMotionData_Curr2CacheHead();
}

//Return relative transformation matrix from received frame to current frame
//void AugmentedVR::calcRelaCamPos(cv::Mat TcwReceived, cv::Mat& Trc){
//    // w: world, r: received frame, c: current frame
////    if (TcwReceived.empty()) return cv::Mat;
//    AVRFrame currFrame;
//    frameCache.getCurrentFrame(currFrame);
//    const cv::Mat Rcw = currFrame.CamMotionMat.rowRange(0,3).colRange(0,3);
//    const cv::Mat tcw = currFrame.CamMotionMat.rowRange(0,3).col(3);
//    const cv::Mat Rwc = Rcw.t();
//    const cv::Mat twc = -Rwc*tcw;
//
//    const cv::Mat Rrw = TcwReceived.rowRange(0,3).colRange(0,3);
//    const cv::Mat trw = TcwReceived.rowRange(0,3).col(3);
//
//    cv::Mat trc = Rrw*twc+trw;
//    cv::Mat Rrc = Rrw*Rwc;
//
//    Trc = cv::Mat::eye(4,4,CV_32F);
//    trc.copyTo(Trc.rowRange(0,3).col(3));
//    Rrc.copyTo(Trc.rowRange(0,3).colRange(0,3));
//    if (DEBUG){
//        cout << "Tcw Received: \n" << TcwReceived << endl;
//        cout << "trc: \n" << trc << endl;
//        cout << "Rrc: \n" << Rrc << endl; // to see if Rlc is almost identity matrix
//    }
////    return trc; // only trc for now, TODO: include Rrc after you can compute perelement 1 by 3 multiplication
////    return Trc;
//}

void AugmentedVR::calcRelaCamPos(cv::Mat TcwReceived, cv::Mat& Tcr){
    // w: world, r: received frame, c: current frame
//    if (TcwReceived.empty()) return cv::Mat;
    AVRFrame currFrame;
    frameCache.getCurrentFrame(currFrame);

    cv::Mat Tcw;
    currFrame.CamMotionMat.copyTo(Tcw);
    Tcr = Tcw * TcwReceived.inv();

    cv::Mat tcr= Tcr.rowRange(0,3).col(3);
    cv::Mat Rcr = Tcr.rowRange(0,3).colRange(0,3);
//    if (DEBUG){
//        cout << "Tcw Received: \n" << TcwReceived << endl;
//        cout << "tcr: \n" << tcr  << endl;
//        cout << "Rcr: \n" << Rcr << endl; // to see if Rlc is almost identity matrix
//    }
//    return trc; // only trc for now, TODO: include Rrc after you can compute perelement 1 by 3 multiplication
//    return Trc;
}

void AugmentedVR::transformRxPCtoMyFrameCoord(cv::Mat Trc, cv::Mat PCReceived, cv::Mat & ret){
//    cout << Trc << endl;
//    debugPC(PCReceived);
//    shiftPC(PCReceived,cv::Scalar(5.,0.,0.));
    transformPC_Via_TransformationMatrix(Trc, PCReceived, ret);
//    debugPC(ret);
}
cv::Mat AugmentedVR::translateRxPCtoMyFrameCoord(cv::Mat trc, cv::Mat PCReceived){

    cv::Scalar trc_vec = cv::Scalar(trc.at<float>(0,0),-trc.at<float>(1,0),-trc.at<float>(2,0),0);
//    cv::Mat trc_mat(height, width, CV_32FC4, trc_vec);
//    cv::Mat transformedPCReceived = PCReceived-trc_vec;
    return PCReceived-trc_vec;
//    transformedPCReceived.copyTo(transRxDynamicPC);

//    if (DEBUG){
//        cout << "trc: " << trc_mat(cv::Rect(dbx,dby,1,1)) << endl;
//        cout << "PCReceived: " << PCReceived(cv::Rect(dbx,dby,1,1)) << endl;
//        cout << "transformedPCReceived: " << transformedPCReceived(cv::Rect(dbx,dby,1,1)) << endl;
//    }

//    return transformedPCReceived;

}


cv::Mat AugmentedVR::removeOverlap(cv::Mat transPC){


    cv::Mat channel[3];
    for (int i=0;i<3;i++){
        extractChannel(transPC,channel[i],i);
    }
    double fov_x,fov_y;
    fov_x = 90; fov_y = 62;
    cv::Mat x_max,y_max;
    x_max = -channel[2]* tan(fov_x/2 * PI / 180.0 );
    y_max = -channel[2] * tan(fov_y/2* PI / 180.0 );
//    cout << tan(30 * PI / 180.0);
//    cout << "x"
//    cout << x_max(Rect(640,360,3,3));
//    cout << y_max(Rect(640,360,3,3));
//    cout << -channel[2](Rect(640,360,3,3));
//    cout << -channel[0](Rect(640,360,3,3));
//    cout << -channel[1](Rect(640,360,3,3));

    cv::Mat mask;
    mask = (channel[0] > x_max) | (channel[0] < -x_max);
    mask |= (channel[1] > y_max) | (channel[1] < -y_max);

//    cout << mask(Rect(640,360,3,3));
    cv::Mat ret;
    transPC.copyTo(ret,mask);

//    if (DEBUG==0){
    cv::Mat chan4;
    extractChannel(transPC,chan4,3);
    cv::Mat ret_chan;
    chan4.copyTo(ret_chan,mask);

    int before, after;
//    cv::Mat nonNaN = cv::Mat(channel[0] != channel[0]);
    before = cv::countNonZero(chan4);
//    cv::Mat nonNaNa = cv::Mat(ret_chan != ret_chan);
    after = cv::countNonZero(ret_chan);
    cout    << "NonZero elements: " << before << endl; // can only count single channel matrix
    cout    << "NonZero elements after FOV removal: " << after<< endl; // can only count single channel matrix

    std::fstream fid("remove_ratio.txt", std::fstream::out | std::fstream::app);
    fid << before << ", " << after << endl;
    fid.close();
//    }

    return ret;
}

void AugmentedVR::dead_reckoning_onRxDynamicPC(){
    cv::Mat tmp;
//    cout << RxMotionVec << endl;
    double x,y,z;
    extractChannel(RxMotionVec,tmp,0);
    x=tmp.at<float>(0,0) / (ZEDCACHESIZE-1);
    extractChannel(RxMotionVec,tmp,1);
    y=tmp.at<float>(0,0) / (ZEDCACHESIZE-1);
    extractChannel(RxMotionVec,tmp,2);
    z=tmp.at<float>(0,0) / (ZEDCACHESIZE-1);
    cout << "dead-reckoning: " << x <<"," << y<<"," <<z << endl;
    cv::Scalar mv(x,y,z);

    transRxDynamicPC -= mv;
}



//TODO: rethink Segmentation!!!

//void AugmentedVR::CheckConnection(int tgt_x, int tgt_y, int cur_x, int cur_y, int idx, std::queue<cv::Point2f> & Q, cv::Mat & checkFlag,cv::Mat & inQueue, float motionThreshold){
//
//
//
//    cv::Mat lastPC, curPC;
//    frameCache.fifo[0].PC_noColor.copyTo(lastPC);
//    frameCache.CurrentFrame.PC_noColor.copyTo(curPC);
//    if (DEBUG) cout << "    Checking " << tgt_x << "," << tgt_y << ": " << (int)frameCache.fifo[0].MotionMask.at<uchar>(tgt_y, tgt_x);
//    if ((int)frameCache.fifo[0].MotionMask.at<uchar>(tgt_y, tgt_x) == 255) {
////    if (PCDisplacement.at<float>(tgt_x,tgt_y) < motionThreshold) {
//        if (DEBUG) cout << " ... in mask\n";
//        // see if it has already been checked
//        if ((int)checkFlag.at<uchar>(tgt_y, tgt_x)==0){
//            // if not push back , expand the search area into the queue, as long as it's within the motionmask
//            if ((int)inQueue.at<uchar>(tgt_y, tgt_x) == 0){
//                if (DEBUG) cout << "        pushing back " << tgt_x << "," << tgt_y << endl;
//                inQueue.at<uchar>(tgt_y, tgt_x) = 255;
//                Q.push(cv::Point2f(tgt_x, tgt_y));
//            }
//        }
//
//        // for each cur node in the queue. check in motion mask
//        cout << (int)frameCache.fifo[0].MotionMask.at<uchar>(cur_y, cur_x) << endl;
//        if ((int)frameCache.fifo[0].MotionMask.at<uchar>(cur_y, cur_x) ==0){
////        if (PCDisplacement.at<float>(tgt_x,tgt_y) >= motionThreshold) {
//            // if not in motionmask check if close to the target which is in motion mask
////            cout << endl << "norm" << norm(lastPC.at<Vec3f>(tgt_x,tgt_y), lastPC.at<Vec3f>(cur_x,cur_y)) << endl;
//
//            if (DEBUG){
//
//                int range = 3;
//                cout << "Last PC: "<<lastPC(cv::Rect(tgt_y, tgt_x,range,range)) << ", " << lastPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
//                cout << "Cur PC: "<<curPC(cv::Rect(tgt_y, tgt_x,range,range)) << ", " << curPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
//                cout << "distance in last PC" << norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
//                cout << "distance in cur PC" << norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
//            }
//
//            if ( min(norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))), norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1)))) < PCConnectionThresh * PATCHSIZE *1.414 ){
//                // mark motion mask if connected
//                if (DEBUG) cout << "        close enough...Adding to mask " << cur_x << "," << cur_y << endl;
////                lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(cur_y, cur_x) = 255;
//
//                frameCache.fifo[0].MotionMask(cv::Rect(cv::Point2f(cur_x, cur_y), cv::Point2f(tgt_x, tgt_y)))   = 255;
//                frameCache.fifo[0].MotionMask(cv::Rect(cur_x, cur_y,1,1))   = 255;
//                // recheck the connections
//                if (cur_x>PATCHSIZE && cur_y>PATCHSIZE)                    CheckConnection(cur_x-PATCHSIZE, cur_y-PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue, motionThreshold);
//                if (cur_x<frameCache.CurrentFrame.FrameLeft.cols-PATCHSIZE && cur_y>PATCHSIZE)     CheckConnection(cur_x+PATCHSIZE, cur_y-PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
//                if (cur_x>PATCHSIZE && cur_y<frameCache.CurrentFrame.FrameLeft.rows-PATCHSIZE)                    CheckConnection(cur_x-PATCHSIZE, cur_y+PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
//                if (cur_x<frameCache.CurrentFrame.FrameLeft.cols-PATCHSIZE && cur_y<frameCache.CurrentFrame.FrameLeft.rows-PATCHSIZE)     CheckConnection(cur_x+PATCHSIZE, cur_y+PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
//            }else{
//                if (DEBUG) cout << "too far \n";
//
//            }
//        }else{
//            frameCache.fifo[0].MotionMask(cv::Rect(cv::Point2f(cur_x, cur_y), cv::Point2f(tgt_x, tgt_y)))   = 255;
//        }
//    }else{
//        if (DEBUG) cout << "..."<< frameCache.fifo[0].MotionMask.at<uchar>(tgt_y, tgt_x) << " ... NOT in mask\n";
//        if ((int)frameCache.fifo[0].MotionMask.at<uchar>(cur_y, cur_x) == 255){
////        if (PCDisplacement.at<float>(cur_x, cur_y) < motionThreshold) {
////            // check if calculations are correct -- checked good code
////            cout << "Point 1: ";
////            for (int i=0;i<3;i++){
////                cout << lastPC.at<Vec3f>(tgt_x,tgt_y)[i] << ", ";
////            }
////            cout << endl << "Point 2: ";
////            for (int i=0;i<3;i++){
////                cout << lastPC.at<Vec3f>(cur_x,cur_y)[i] << ", ";
////            }
////            cout << endl << "norm" << norm(lastPC.at<Vec3f>(tgt_x,tgt_y), lastPC.at<Vec3f>(cur_x,cur_y)) << endl;
//            if (DEBUG){
//
//                int range = 3;
//                cout << "Last PC: "<<lastPC(cv::Rect(tgt_y, tgt_x,range,range)) << endl << lastPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
//                cout << "Cur PC: "<<curPC(cv::Rect(tgt_y, tgt_x,range,range)) << endl << curPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
//                cout << "distance in last PC" << norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
//                cout << "distance in cur PC" << norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
//            }
//
//            if ( min(norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))), norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1)))) < PCConnectionThresh * PATCHSIZE *1.414 ){
//                // see if it has already been checked
//                if ((int)checkFlag.at<uchar>(tgt_y, tgt_x)==0){
//                    // if not push back , expand the search area into the queue, as long as it's within the motionmask
//
//                    if ((int)inQueue.at<uchar>(tgt_y, tgt_x) == 0){
//                        if (DEBUG) cout << "        close enough...pushing back " << tgt_x << "," << tgt_y << endl;
//                        inQueue.at<uchar>(tgt_y, tgt_x) = 255;
//                        Q.push(cv::Point2f(tgt_x, tgt_y));
//                    }
//                }
//            }else{
//                if (DEBUG) cout << "too far \n";
//
//            }
//        }
//    }
//}








void AugmentedVR::ObjectMotionAnalysis(){


    vector<cv::Point2f> points_trans;
    cv::Mat img;
    AVRFrame cur;
    frameCache.getCurrentFrame(cur);
//    cur.FrameLeft.copyTo(img);

//    img = mSLAM->DrawSlamFrame();
    AVRFrame cacheHead;
    frameCache.getFIFOHead(cacheHead);
    cacheHead.FrameLeft.copyTo(img);

    if (cur.sceneTransformMat_Curr2CacheHead.empty() || cur.MotionMask.empty()){
        std::cerr << "ObjectMotionAnalysis: Miss\n";
        return;
    }

    if (cacheHead.tracked_keypoints.size()!= cur.tracked_keypoints.size()){
        std::cerr << "Keypoints Size Mismatch\n";
        return;
    }
    perspectiveTransform( cur.tracked_keypoints, points_trans, cur.sceneTransformMat_Curr2CacheHead);

    cv::Mat total_motionVec(1,1,CV_32FC3,cv::Scalar(0.,0.,0.));
    int count = 0;

    vector<xyzNorm> mv;
    for( size_t i = 0; i < points_trans.size(); i++ ) {
        cv::Rect rect(0, 0, img.cols, img.rows);
        if (rect.contains(cur.tracked_keypoints[i])  && rect.contains(cacheHead.tracked_keypoints[i])) {
            // check if the point is in range after transformation
            if (cur.tracked_status[i] && norm(points_trans[i] - cacheHead.tracked_keypoints[i]) > 5) {
                if (cur.MotionMask.at<uchar>(cacheHead.tracked_keypoints[i]) == 255 &&
                        cur.MotionMask.at<uchar>(cur.tracked_keypoints[i]) == 255) {
//                    if (DEBUG) {
//                        drawMatchedKeypoints(img,cacheHead.tracked_keypoints[i],cur.tracked_keypoints[i], to_string(i));
////                        circle(img, cacheHead.tracked_keypoints[i], 3, cv::Scalar(0, 0, 255), -1, 8);
////                        circle(img, points_trans[i], 3, cv::Scalar(255, 0, 0), -1, 8);
////                        circle(img, cur.tracked_keypoints[i], 3, cv::Scalar(0, 255, 0), -1, 8);
////                        line(img, points_trans[i], cur.tracked_keypoints[i], cv::Scalar(0, 0, 0));
////                        line(img, cacheHead.tracked_keypoints[i], points_trans[i],
////                             cv::Scalar(0, 0, 0));
////                        cv::putText(img, to_string(i), cur.tracked_keypoints[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
//                    }
                    cv::Mat motionVec = cur.PC_noColor(cv::Rect(int(cur.tracked_keypoints[i].x), int(cur.tracked_keypoints[i].y), 1, 1))
                                        -cacheHead.PC_noColor(cv::Rect(int(cacheHead.tracked_keypoints[i].x),int(cacheHead.tracked_keypoints[i].y), 1, 1));

                    //// get speed motionvec
                    motionVec /= double(cur.frameTS-cacheHead.frameTS);

                    double dist = norm(motionVec);
                    if (cvIsInf(dist) || cvIsNaN(dist)) continue;
                    if (DEBUG) cout << "Point " << i << ": " << motionVec << " >> " << dist << endl;
                    total_motionVec += motionVec;

                    count++;

                    /// record the motion vector
                    xyzNorm xyzd = {motionVec.at<float>(0,0), motionVec.at<float>(0,1),motionVec.at<float>(0,2),dist};
                    mv.push_back(xyzd);
                }
            }
        }
    }



    total_motionVec /= count;

//    cv::Mat filteredMotionVec(1,1,CV_32FC3,cv::Scalar(0.,0.,0.));
    float filteredMotionVec[3] = {0.,0.,0.};
    double avgDist = norm(total_motionVec);
    int filterCount = 0;
    for (int i=0;i<mv.size();i++){
        if (mv[i].norm < avgDist * 1.6 && mv[i].norm > avgDist*0.4){
            if (DEBUG) {
                drawMatchedKeypoints(img,cacheHead.tracked_keypoints[i],cur.tracked_keypoints[i], to_string(i));
            }
            filteredMotionVec[0]+= mv[i].x;
            filteredMotionVec[1]+= mv[i].y;
            filteredMotionVec[2]+= mv[i].z;
            filterCount++;
        }
    }
    filteredMotionVec[0]/=filterCount;
    filteredMotionVec[1]/=filterCount;
    filteredMotionVec[2]/=filterCount;
    cv::Mat avg_filterMotionVec = cv::Mat(1,1,CV_32FC3,cv::Scalar(filteredMotionVec[0],filteredMotionVec[1],filteredMotionVec[2]));

    if (DEBUG && VISUAL){
        cv::Mat masked_img;
        img.copyTo(masked_img);
        imshow("masked KLT matches", masked_img);
    }
    if (DEBUG){
        cout << "Count: " << count << endl;
        cout << "Total Motion Vec: " << total_motionVec << " >> " << norm(total_motionVec)<< endl;
        cout << "FilterCount: " << filterCount << endl;
        cout << "Total Filter Motion Vec: " << avg_filterMotionVec << " >> " << norm(avg_filterMotionVec)<< endl;
    }

    total_motionVec.copyTo(ObjectMotionVec);
    // TODO: find a safe way to do it
    ObjectMotionVec.copyTo(frameCache.CurrentFrame.ObjectMotionVec);
    avg_filterMotionVec.copyTo(frameCache.CurrentFrame.FilteredObjectMotionVec);
    /// low pass filtering (sliding window average)
    frameCache.getLowPassMotionVectorForCurrFrame();
    frameCache.getLowPassFilteredMotionVectorForCurrFrame();
}

void AugmentedVR::TransPCvsPC(cv::Mat& rxTcw, cv::Mat& rxFrame, cv::Mat& rxMV, int rxTS){
    static AVRFrame cur;
    frameCache.getCurrentFrame(cur);
    if (rxTcw.empty() || rxFrame.empty() || transRxPC.empty() || cur.MotionMask.empty()) return;
    cv::Mat RxFrameGray;
    cv::cvtColor(rxFrame,RxFrameGray,cv::COLOR_RGB2GRAY);
    vector<cv::Point2f> kpReceived, kp_oppo;
    std::vector<uchar> status, status_oppo;
    std::vector<float> error;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size  winSize(31, 31);

    cv::calcOpticalFlowPyrLK(cur.FrameLeftGray, RxFrameGray, cur.keypoints, kpReceived,
                             status, error, winSize, 3, termcrit, 0, 0.001);
    cv::calcOpticalFlowPyrLK(RxFrameGray, cur.FrameLeftGray, kpReceived, kp_oppo,
                             status_oppo, error, winSize, 3, termcrit, 0, 0.001);
    cv::Mat img;
    cur.FrameLeft.copyTo(img);

    for (int i=0;i<kpReceived.size();i++){
//        if (status[i] && status_oppo[i] && cur.MotionMask.at<uchar>(cur.keypoints[i])!=255 ){ /// statics
        if (status[i] && status_oppo[i] && cur.MotionMask.at<uchar>(cur.keypoints[i])==255 ){ /// dynamics
            drawMatchedKeypoints(img, cur.keypoints[i], kpReceived[i], to_string(i));
            drawMatchedKeypoints(rxFrame, cur.keypoints[i], kpReceived[i], to_string(i));
            cv::Scalar diff = cur.pointcloud.at<cv::Vec4f>(cur.keypoints[i]) - transRxPC.at<cv::Vec4f>(kpReceived[i]);
            cv::Scalar dff3 = cv::Scalar(diff[0],diff[1],diff[2]);
            if (DEBUG){
                cout << "Point " << i << ": "
//                     << cur.pointcloud.at<cv::Vec4f>(cur.keypoints[i]) << " - "
//                     << transRxPC.at<cv::Vec4f>(kpReceived[i]) << " = "
//                     << endl
                     << dff3 << " >>> " << norm(dff3) << endl;
                imshow("TranPCvsPC_Curr", img);
                cv::setMouseCallback("TranPCvsPC_Curr", onMouseCallback_DisplayVoxel, &(cur.pointcloud));
                imshow("TranPCvsPC_Rx", rxFrame);
                cv::setMouseCallback("TranPCvsPC_Rx", onMouseCallback_DisplayVoxel, &(transRxPC));
            }
        }
    }


    /// time diff
    if (DEBUG){

        cout << "Current TS: " << cur.frameTS << endl;
        cout << "Rx TS:" << rxTS << endl;
        cout << "TS diff: " << int(cur.frameTS)-rxTS << endl;
//        debugPC(transRxPC);
    }
    LatencyCompensation(rxMV, transRxPC, int(cur.frameTS)-rxTS);
//    if (DEBUG)debugPC(transRxPC);
}

//void AugmentedVR::TransPCvsPC(){
//    AVRFrame cur;
//    frameCache.getCurrentFrame(cur);
//    if (RxTCW.empty() || RxFrame.empty() || transRxPC.empty() || cur.MotionMask.empty()) return;
//    cv::Mat RxFrameGray;
//    cv::cvtColor(RxFrame,RxFrameGray,cv::COLOR_RGB2GRAY);
//    vector<cv::Point2f> kpReceived;
//    std::vector<uchar> status;
//    std::vector<float> error;
//    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
//    cv::Size  winSize(31, 31);
//
//    cv::calcOpticalFlowPyrLK(cur.FrameLeftGray, RxFrameGray, cur.keypoints, kpReceived,
//                             status, error, winSize, 3, termcrit, 0, 0.001);
//    cv::Mat img;
//    cur.FrameLeft.copyTo(img);
//
//    for (int i=0;i<kpReceived.size();i++){
//        if (status[i] && cur.MotionMask.at<uchar>(cur.keypoints[i])==255 ){
//            drawMatchedKeypoints(img, cur.keypoints[i], kpReceived[i], to_string(i));
//            drawMatchedKeypoints(RxFrame, cur.keypoints[i], kpReceived[i], to_string(i));
//            cv::Scalar diff = cur.pointcloud.at<cv::Vec4f>(cur.keypoints[i]) - transRxPC.at<cv::Vec4f>(kpReceived[i]);
//            cv::Scalar dff3 = cv::Scalar(diff[0],diff[1],diff[2]);
//            cout << "Point " << i << ": "
//                 << cur.pointcloud.at<cv::Vec4f>(cur.keypoints[i]) << " - "
//                 << transRxPC.at<cv::Vec4f>(kpReceived[i]) << " = "
//                 << endl
//                 << dff3 << " >>> " << norm(dff3) << endl;
//            imshow("TranPCvsPC_Curr", img);
//            imshow("TranPCvsPC_Rx", RxFrame);
//        }
//    }
//
//
//    /// time diff
//    if (DEBUG){
//
//        cout << "Current TS: " << cur.frameTS << endl;
//        cout << "Rx TS:" << RxTimeStamp << endl;
//        cout << "TS diff: " << int(cur.frameTS)-RxTimeStamp << endl;
//        debugPC(transRxPC);
//    }
//    LatencyCompensation(RxMotionVec, transRxPC, int(cur.frameTS)-RxTimeStamp);
//    if (DEBUG)debugPC(transRxPC);
//}


void AugmentedVR::LatencyCompensation(cv::Mat& MotionVec, cv::Mat& PC, int TSdiff){

    cv::Mat tmp;
//    cout << RxMotionVec << endl;
    double x,y,z;
    extractChannel(MotionVec,tmp,0);
    x=tmp.at<float>(0,0);
    extractChannel(MotionVec,tmp,1);
    y=tmp.at<float>(0,0);
    extractChannel(MotionVec,tmp,2);
    z=tmp.at<float>(0,0);
    if (DEBUG)cout << "MotionVec (m/us): " << x <<"," << y<<"," <<z << endl;
    cv::Scalar mv(x,y,z);


    PC += mv*double(TSdiff);
    if (DEBUG)cout << "LatencyCompensation: " << mv*double(TSdiff) << endl;
}