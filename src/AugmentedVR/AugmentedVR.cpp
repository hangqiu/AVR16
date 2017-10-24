//
// Created by nsl on 9/20/16.
//



#include "AugmentedVR.hpp"
#include <ctime>
#include <sys/time.h>
#include <include/UtilsPC.hpp>

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
        return 1;
    }
    Resolution image_size = mZEDCam->getResolution();
    width = (int)image_size.width;
    height = (int)image_size.height;
//    mZEDCam->setConfidenceThreshold(ZEDConfidence);

    int tmp_id = 0;
    while (tmp_id++ < startFrameID){
        cerr << "skipping frame " << tmp_id << endl;
        grabNextZEDFrameOffline();
    }
    grabNextZEDFrameOffline();
    if (!frameCache.NextFrame2SlamFrame()){
        if (DEBUG) cerr<< " AugmentedVR::initZEDCam: SlamFrame2CurrentFrame Failure\n";
    };
    grabNextZEDFrameOffline();

    // Here starts the frame seq 0.
    TotalFrameSeq = 0;
    startTS = frameCache.getLatestTS();
    frameCache.setStartTS(startTS);



    return 0;
}


void AugmentedVR::initSLAMStereo(string VocFile, string CalibrationFile, bool bReuseMap, string mapFile){

    if (bReuseMap == true){
        mSLAM = new ORB_SLAM2::System(VocFile,CalibrationFile,ORB_SLAM2::System::STEREO,true, bReuseMap, mapFile);
    }else{
        mSLAM = new ORB_SLAM2::System(VocFile,CalibrationFile,ORB_SLAM2::System::STEREO,true, bReuseMap);
    }
    FeedSlamNextFrame();
}

bool AugmentedVR::grabNextZEDFrameOffline() {

    if  (mZEDCam->grab(runtimeParameters) != SUCCESS){
        cerr << "AugmentedVR::grabNextZEDFrameOffline: can't grab image" << endl;
        return false;
    }

    frameCache.LoadNextFrameFromZED(mZEDCam, width, height, TotalFrameSeq++);

    if (!INIT_FLAG) {
        frameCache.getNextFrame().pointcloud.copyTo(initPC);//TODO
        INIT_FLAG = true;
    }

    return true;
}

void AugmentedVR::FeedSlamNextFrame(){
#ifdef EVAL
    timeval start,end;
    gettimeofday(&start, NULL);
#endif
    mSLAM->mTracker_CreateNextFrame(frameCache.getSlamFrame().FrameLeftGray,
                                    frameCache.getSlamFrame().FrameRightGray,
                                    frameCache.getSlamFrame().frameTS);
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "FeedSlamNextFrame >>>>>>> Load Next Frame: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
#endif
}


AVRFrame AugmentedVR::getCurrentAVRFrame(){
    return frameCache.getCurrentFrame();
}

void AugmentedVR::SinkFrames(){
    frameCache.SinkFrames();
}

bool AugmentedVR::PrepareNextFrame() {


#ifdef PIPELINE
    thread prefetch(&AugmentedVR::grabNextZEDFrameOffline,this);
#else
    if (!grabNextZEDFrameOffline()) return false;
#endif

    // create next frame for slam as well...
    // use gray to save conversion time.
    // thread feedslam(&AugmentedVR::FeedSlamNextFrame,this);
    FeedSlamNextFrame();

#ifdef PIPELINE
    prefetch.join();
#endif
    return true;
}

void AugmentedVR::calcOpticalFlow(){
    if (!initialized){
        frameCache.updateCurrFrameFeature();
        initialized = true;
        return;
    }
    if (FRAME_ID%DUTYCYCLE==0){
        frameCache.updateLastFrameFeature();

    }
    frameCache.opticalFlowTrack_Curr2Last();
}



// call SLAM and calculate camera pose relative to last frame
// update rotation and translation matrxi to last frame: Rlc, tlc
void AugmentedVR::trackCam() {
    AVRFrame currFrame = getCurrentAVRFrame();
    (mSLAM->TrackStereo(currFrame.FrameLeft,
                        currFrame.FrameRight,
                        currFrame.frameTS))
            .copyTo(frameCache.CurrentFrame.CamMotionMat); //TODO: conflict with FeedSlamNextFrame?
    // for debug only
//    if(COOP){
//        CamMotionMat.at<float>(0,3) +=7;
//    }
    if (SHOW_CAMMOTION) cout << "CamMotionMat: \n" << frameCache.CurrentFrame.CamMotionMat << endl;
}

bool AugmentedVR::trackGood(){
    return !(getCurrentAVRFrame().CamMotionMat.empty());
}

void AugmentedVR::analyze(){
#ifdef EVAL
    timeval tTotalStart, tFetchStart, tCacheStart, tSlamStart, tPCMotionStart, tPCMotionFilterStart, tObjectFilterStart, tTXStart, tRXStart, tPCmergeStart,tDeadReckonStart;
    timeval tTotalEnd, tFetchEnd, tCacheEnd, tSlamEnd, tPCMotionEnd, tPCMotionFilterEnd, tObjectFilterEnd, tTXEnd, tRXEnd, tPCmergeEnd, tDeadReckonEnd;
#endif
    if (!frameCache.ReachFullMotionBacklog()){
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
    ObjectMotionAnalysis(ZEDCACHESIZE - 1);
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
    assert(frameCache.ReachFullMotionBacklog());
    frameCache.updateMotionData_Curr2CacheHead();
}

//Return relative transformation matrix from received frame to current frame
cv::Mat AugmentedVR::calcRelaCamPos(cv::Mat TcwReceived){
    // w: world, r: received frame, c: current frame
//    if (TcwReceived.empty()) return cv::Mat;
    AVRFrame currFrame = frameCache.getCurrentFrame();
    const cv::Mat Rcw = currFrame.CamMotionMat.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = currFrame.CamMotionMat.rowRange(0,3).col(3);
    const cv::Mat Rwc = Rcw.t();
    const cv::Mat twc = -Rwc*tcw;

    const cv::Mat Rrw = TcwReceived.rowRange(0,3).colRange(0,3);
    const cv::Mat trw = TcwReceived.rowRange(0,3).col(3);

    cv::Mat trc = Rrw*twc+trw;
    cv::Mat Rrc = Rrw*Rwc;

    cv::Mat Trc = cv::Mat::eye(4,4,CV_32F);
    trc.copyTo(Trc.rowRange(0,3).col(3));
    Rrc.copyTo(Trc.rowRange(0,3).colRange(0,3));
//    if (DEBUG){
        cout << "trc: \n" << trc << endl;
        cout << "Rrc: \n" << Rrc << endl; // to see if Rlc is almost identity matrix
//    }
//    return trc; // only trc for now, TODO: include Rrc after you can compute perelement 1 by 3 multiplication
    return Trc;
}

cv::Mat AugmentedVR::transformRxPCtoMyFrameCoord(cv::Mat Trc, cv::Mat PCReceived){
    cout << Trc << endl;
    debugPC(PCReceived);
//    if (DEBUG){
//        timeval start,end;
//        gettimeofday(&start, NULL);
//    }
    cv::Mat ret = transformPCViaTransformationMatrix_gpu(Trc, PCReceived);
//    if (DEBUG){
//        gettimeofday(&end,NULL);
//        cout << "Transformation time: " << double(end.tv_sec-start.tv_sec)*1000
//             << double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//
//    }
    debugPC(ret);
    return ret;
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

void AugmentedVR::CheckConnection(int tgt_x, int tgt_y, int cur_x, int cur_y, int idx, std::queue<cv::Point2f> & Q, cv::Mat & checkFlag,cv::Mat & inQueue, float motionThreshold){



    cv::Mat lastPC, curPC;
    frameCache.fifo[0].PC_noColor.copyTo(lastPC);
    frameCache.CurrentFrame.PC_noColor.copyTo(curPC);
    if (DEBUG) cout << "    Checking " << tgt_x << "," << tgt_y << ": " << (int)frameCache.fifo[0].MotionMask.at<uchar>(tgt_y, tgt_x);
    if ((int)frameCache.fifo[0].MotionMask.at<uchar>(tgt_y, tgt_x) == 255) {
//    if (PCDisplacement.at<float>(tgt_x,tgt_y) < motionThreshold) {
        if (DEBUG) cout << " ... in mask\n";
        // see if it has already been checked
        if ((int)checkFlag.at<uchar>(tgt_y, tgt_x)==0){
            // if not push back , expand the search area into the queue, as long as it's within the motionmask
            if ((int)inQueue.at<uchar>(tgt_y, tgt_x) == 0){
                if (DEBUG) cout << "        pushing back " << tgt_x << "," << tgt_y << endl;
                inQueue.at<uchar>(tgt_y, tgt_x) = 255;
                Q.push(cv::Point2f(tgt_x, tgt_y));
            }
        }

        // for each cur node in the queue. check in motion mask
        cout << (int)frameCache.fifo[0].MotionMask.at<uchar>(cur_y, cur_x) << endl;
        if ((int)frameCache.fifo[0].MotionMask.at<uchar>(cur_y, cur_x) ==0){
//        if (PCDisplacement.at<float>(tgt_x,tgt_y) >= motionThreshold) {
            // if not in motionmask check if close to the target which is in motion mask
//            cout << endl << "norm" << norm(lastPC.at<Vec3f>(tgt_x,tgt_y), lastPC.at<Vec3f>(cur_x,cur_y)) << endl;

            if (DEBUG){

                int range = 3;
                cout << "Last PC: "<<lastPC(cv::Rect(tgt_y, tgt_x,range,range)) << ", " << lastPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
                cout << "Cur PC: "<<curPC(cv::Rect(tgt_y, tgt_x,range,range)) << ", " << curPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
                cout << "distance in last PC" << norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
                cout << "distance in cur PC" << norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
            }

            if ( min(norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))), norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1)))) < PCConnectionThresh * PATCHSIZE *1.414 ){
                // mark motion mask if connected
                if (DEBUG) cout << "        close enough...Adding to mask " << cur_x << "," << cur_y << endl;
//                lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(cur_y, cur_x) = 255;

                frameCache.fifo[0].MotionMask(cv::Rect(cv::Point2f(cur_x, cur_y), cv::Point2f(tgt_x, tgt_y)))   = 255;
                frameCache.fifo[0].MotionMask(cv::Rect(cur_x, cur_y,1,1))   = 255;
                // recheck the connections
                if (cur_x>PATCHSIZE && cur_y>PATCHSIZE)                    CheckConnection(cur_x-PATCHSIZE, cur_y-PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue, motionThreshold);
                if (cur_x<frameCache.CurrentFrame.FrameLeft.cols-PATCHSIZE && cur_y>PATCHSIZE)     CheckConnection(cur_x+PATCHSIZE, cur_y-PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
                if (cur_x>PATCHSIZE && cur_y<frameCache.CurrentFrame.FrameLeft.rows-PATCHSIZE)                    CheckConnection(cur_x-PATCHSIZE, cur_y+PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
                if (cur_x<frameCache.CurrentFrame.FrameLeft.cols-PATCHSIZE && cur_y<frameCache.CurrentFrame.FrameLeft.rows-PATCHSIZE)     CheckConnection(cur_x+PATCHSIZE, cur_y+PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
            }else{
                if (DEBUG) cout << "too far \n";

            }
        }else{
            frameCache.fifo[0].MotionMask(cv::Rect(cv::Point2f(cur_x, cur_y), cv::Point2f(tgt_x, tgt_y)))   = 255;
        }
    }else{
        if (DEBUG) cout << "..."<< frameCache.fifo[0].MotionMask.at<uchar>(tgt_y, tgt_x) << " ... NOT in mask\n";
        if ((int)frameCache.fifo[0].MotionMask.at<uchar>(cur_y, cur_x) == 255){
//        if (PCDisplacement.at<float>(cur_x, cur_y) < motionThreshold) {
//            // check if calculations are correct -- checked good code
//            cout << "Point 1: ";
//            for (int i=0;i<3;i++){
//                cout << lastPC.at<Vec3f>(tgt_x,tgt_y)[i] << ", ";
//            }
//            cout << endl << "Point 2: ";
//            for (int i=0;i<3;i++){
//                cout << lastPC.at<Vec3f>(cur_x,cur_y)[i] << ", ";
//            }
//            cout << endl << "norm" << norm(lastPC.at<Vec3f>(tgt_x,tgt_y), lastPC.at<Vec3f>(cur_x,cur_y)) << endl;
            if (DEBUG){

                int range = 3;
                cout << "Last PC: "<<lastPC(cv::Rect(tgt_y, tgt_x,range,range)) << endl << lastPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
                cout << "Cur PC: "<<curPC(cv::Rect(tgt_y, tgt_x,range,range)) << endl << curPC(cv::Rect(cur_y, cur_x,range,range)) << endl;
                cout << "distance in last PC" << norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
                cout << "distance in cur PC" << norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1))) << endl;
            }

            if ( min(norm(lastPC(cv::Rect(tgt_y, tgt_x,1,1))-lastPC(cv::Rect(cur_y, cur_x,1,1))), norm(curPC(cv::Rect(tgt_y, tgt_x,1,1))- curPC(cv::Rect(cur_y, cur_x,1,1)))) < PCConnectionThresh * PATCHSIZE *1.414 ){
                // see if it has already been checked
                if ((int)checkFlag.at<uchar>(tgt_y, tgt_x)==0){
                    // if not push back , expand the search area into the queue, as long as it's within the motionmask

                    if ((int)inQueue.at<uchar>(tgt_y, tgt_x) == 0){
                        if (DEBUG) cout << "        close enough...pushing back " << tgt_x << "," << tgt_y << endl;
                        inQueue.at<uchar>(tgt_y, tgt_x) = 255;
                        Q.push(cv::Point2f(tgt_x, tgt_y));
                    }
                }
            }else{
                if (DEBUG) cout << "too far \n";

            }
        }
    }
}








void AugmentedVR::ObjectMotionAnalysis(int idx){


    vector<cv::Point2f> points_trans;

    cv::Mat img;

    frameCache.CurrentFrame.FrameLeft.copyTo(img);

//    img = mSLAM->DrawSlamFrame();


    if (!frameCache.fifo[0].sceneTransformMat.empty()&& !frameCache.fifo[0].MotionMask.empty()){

        perspectiveTransform( frameCache.fifo[0].keypoints, points_trans, frameCache.fifo[0].sceneTransformMat);

        cv::Mat total_motionVec(1,1,CV_32FC3,cv::Scalar(0.,0.,0.));
        int count = 0;


        for( size_t i = 0; i < points_trans.size(); i++ ) {
            cv::Rect rect(0, 0, img.cols, img.rows);
            if (rect.contains(frameCache.CurrentFrame.keypoints[i])  && rect.contains(frameCache.fifo[0].keypoints[i])) {
                // check if the point is in range after transformation
                if (frameCache.CurrentFrame.tracked_status[i] && norm(points_trans[i] - frameCache.CurrentFrame.keypoints[i]) > 5) {

                    if (DEBUG) {

                        circle(img, frameCache.fifo[0].keypoints[i], 3, cv::Scalar(255, 255, 0), -1, 8);
                        circle(img, points_trans[i], 3, cv::Scalar(255, 0, 0), -1, 8);
                        circle(img, frameCache.CurrentFrame.keypoints[i], 3, cv::Scalar(0, 255, 0), -1, 8);
                        line(img, points_trans[i], frameCache.CurrentFrame.keypoints[i], cv::Scalar(0, 0, 255));
                        line(img, frameCache.fifo[0].keypoints[i], points_trans[i],
                             cv::Scalar(0, 255, 255));
                    }

                    if (frameCache.fifo[0].MotionMask.at<uchar>(frameCache.fifo[0].keypoints[i]) == 255 &&
                            frameCache.fifo[0].MotionMask.at<uchar>(frameCache.CurrentFrame.keypoints[i]) == 255) {


                        //                    cout << lastStereoData[ZEDCACHESIZE-1-idx].PC_noColor(Rect(lastStereoData[ZEDCACHESIZE-1-idx].keypoints[i].y,lastStereoData[ZEDCACHESIZE-1-idx].keypoints[i].y,1,1));
                        //                    cout << lastStereoData[ZEDCACHESIZE-1-idx].PC_noColor.at<Vec3f>(lastStereoData[ZEDCACHESIZE-1-idx].keypoints[i])  << endl;
                        //                    cout << PC_noColor(Rect(keypoints[i].y,keypoints[i].y,1,1));
                        //                    cout << PC_noColor.at<Vec3f>(keypoints[i]) << endl;


                        cv::Mat motionVec = frameCache.fifo[0].PC_noColor(
                                cv::Rect(frameCache.fifo[0].keypoints[i].x,
                                         frameCache.fifo[0].keypoints[i].y, 1, 1)) -
                                frameCache.CurrentFrame.PC_noColor(cv::Rect(frameCache.CurrentFrame.keypoints[i].x, frameCache.CurrentFrame.keypoints[i].y, 1, 1));
                        double dist = norm(motionVec);
                        if (cvIsInf(dist) || cvIsNaN(dist)) continue;
                        if (DEBUG > 1) cout << "Point " << i << ": " << motionVec << " >> " << dist << endl;
                        total_motionVec += motionVec;
                        //                    cout << motionVec.type();
                        //                    cout << PC_noColor.type();
                        if (DEBUG > 1)
                            cout << "Total Motion Vec: " << total_motionVec << " >> " << norm(total_motionVec) << endl;
                        count++;
                    }
                }
            }
        }

        if (DEBUG && VISUAL){
//            imshow("KLT matches", img);
            cv::Mat masked_img;
            img.copyTo(masked_img,frameCache.fifo[0].MotionMask);
            imshow("masked KLT matches", masked_img);
        }

        total_motionVec /= count;
        if (DEBUG)
            cout << "Total Motion Vec: " << total_motionVec << " >> " << norm(total_motionVec)<< endl;
        total_motionVec.copyTo(ObjectMotionVec);
        // TODO: find a safe way to do it
        ObjectMotionVec.copyTo(frameCache.fifo[0].ObjectMotionVec);

        // low pass filtering (sliding window average)
        cv::Mat lp_total(1,1,CV_32FC3,cv::Scalar(0.,0.,0.));
        for (int i=0;i<ZEDCACHESIZE-1;i++){
            if (!frameCache.fifo[0].ObjectMotionVec.empty()){
                lp_total+= frameCache.fifo[0].ObjectMotionVec;
            }
        }
        lp_total /= ZEDCACHESIZE-1;
        lp_total.copyTo(Log_LowPassMotionVec);
        Log_LowPassMotionVec.copyTo(frameCache.fifo[0].LowPass_ObjectMotionVec);
        if (DEBUG) cout << "Low Pass Total Motion Vec: " << Log_LowPassMotionVec<< " >> " << norm(Log_LowPassMotionVec)<< endl;
    }
}
