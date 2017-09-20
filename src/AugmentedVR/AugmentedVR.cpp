//
// Created by nsl on 9/20/16.
//



#include "AugmentedVR.hpp"

#include <ctime>
#include <sys/time.h>

//#include "opencv2/cuda.hpp"
#include <thread>

//#define EVAL


AugmentedVR::AugmentedVR(sl::zed::SENSING_MODE senseMode, int CamId) {
    ZED_LRes = cv::Mat(DisplaySize, CV_8UC4);

    AugmentedVR::senseMode = senseMode;
    AugmentedVR::CamId = CamId;
    frameTS = 0;
    ZEDTS =0;
    startTS = 0;
    mIo = new IO(this);

    // creating a dummy next frame
//    NextFrame.FrameLeft = cv::Mat(width, height, CV_8UC3, Scalar(255,255,255));
//    NextFrame.FrameRight = cv::Mat(width, height, CV_8UC3, Scalar(255,255,255));
//    NextFrame.FrameLeftGray = cv::Mat(width, height, CV_8U, Scalar(255));
//    NextFrame.FrameRightGray = cv::Mat(width, height, CV_8U, Scalar(255));
//    NextFrame.pointcloud = cv::Mat(width,height,CV_32FC4, Scalar(0.,0.,0.,0.));
//    NextFrame.PC_noColor = cv::Mat(width,height,CV_32FC3, Scalar(0.,0.,0.));
}

AugmentedVR::~AugmentedVR(){
    mSLAM->SaveTrajectoryKITTI("CameraTrajectory.txt");
    mSLAM->Shutdown();
    delete(mZEDCam);
    delete(mSLAM);
    cout << "AVR shuts down" << endl;
}

int AugmentedVR::initZEDCam(const string& videoPath, int startFrameID, InitParams parameters, int ZEDConfidence){

    mZEDCam = new sl::zed::Camera(videoPath);
    sl::zed::ERRCODE err = mZEDCam->init(parameters);
    cout << "ZED N°" << CamId << " -> Result : " << errcode2str(err) << endl;
    if (err != sl::zed::SUCCESS) {
        delete mZEDCam;
        return 1;
    }

    width = mZEDCam->getImageSize().width;
    height = mZEDCam->getImageSize().height;
    SbSResult = cv::Mat(height, width * 2, CV_8UC4, 1);
    // remove the not to be trusted data
    mZEDCam->setConfidenceThreshold(ZEDConfidence);



    int tmp_id = 0;
    while (tmp_id++ < startFrameID){
        grabNextZEDFrameOffline();

//        if (DEBUG){ // TODO change this using ULogger from rtapmap
//            cout << "skipping " << ZEDTS <<  endl;
//        }
        // 	char wnd_name[21];
        //        sprintf(wnd_name, "ZED N° %d", i);
        //        cv::resize(SbSResult[i], ZED_LRes[i], DisplaySize);
        //        cv::imshow(wnd_name, ZED_LRes[i]);
        //    }
    }

    SlamFrame = NextFrame;
    NextFrame = ZED_DATA{};
    grabNextZEDFrameOffline();

    // Here starts the frame seq 0.
    frameSeq = 0;
    startTS = ZEDTS;
    frameTS = 0;

    for (int i=0;i<ZEDCACHESIZE;i++){
        cv::Mat tmp;
        STEREO_DATA stereoTmp;
        lastStereoData.push_back(stereoTmp);
//        lastpointcloud.push_back(tmp);
//        PCMotionVec.push_back(tmp);
    }


    PCDisplacement = cv::Mat(height,width, CV_32F, cv::Scalar(255.0));

    return 0;
}


int AugmentedVR::initZEDCamLive(const sl::zed::ZEDResolution_mode ZED_RES, const int FPS, InitParams parameters,
                                int ZEDConfidence){
    mZEDCam = new sl::zed::Camera(ZED_RES,FPS,CamId);
    sl::zed::ERRCODE err = mZEDCam->init(parameters);
    cout << "ZED N°" << CamId << " -> Result : " << errcode2str(err) << endl;
    if (err != sl::zed::SUCCESS) {
        delete mZEDCam;
        return 1;
    }

    width = mZEDCam->getImageSize().width;
    height = mZEDCam->getImageSize().height;
    SbSResult = cv::Mat(height, width * 2, CV_8UC4, 1);

    // remove the not to be trusted data
    mZEDCam->setConfidenceThreshold(ZEDConfidence);
    return 0;
}

void AugmentedVR::initSLAMStereo(string VocFile, string CalibrationFile, bool bReuseMap, string mapFile){

    if (bReuseMap == true){
//        SLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::STEREO, true, bReuseMap,  argv[5]);
        mSLAM = new ORB_SLAM2::System(VocFile,CalibrationFile,ORB_SLAM2::System::STEREO,true, bReuseMap, mapFile);
    }else{
        mSLAM = new ORB_SLAM2::System(VocFile,CalibrationFile,ORB_SLAM2::System::STEREO,true, bReuseMap);
//        SLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, bReuseMap);
    }
    FeedSlamNextFrame();
}

void AugmentedVR::FeedSlamNextFrame(){
#ifdef EVAL
    timeval start,end;
    gettimeofday(&start, NULL);
#endif
    mSLAM->mTracker_CreateNextFrame(SlamFrame.FrameLeftGray,SlamFrame.FrameRightGray,SlamFrame.frameTS);
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "FeedSlamNextFrame >>>>>>> Load Next Frame: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
#endif
}

//void AugmentedVR::grab_run() {
//    while (!stop_signal) {
//        bool res = zed[x]->grab(SENSING_MODE::FILL, 1, 1, 1);
//        if (!res) {
//            frame_seq[x]++;
//            ZED_Timestamp[x] = zed[x]->getCameraTimestamp();
//            //sl::zed::Mat depthMM = zed[x]->retrieveMeasure(MEASURE::DEPTH);
//            slMat2cvMat(zed[x]->retrieveImage(SIDE::LEFT)).copyTo(SbSResult[x](cv::Rect(0, 0, width, height)));
//            slMat2cvMat(zed[x]->normalizeMeasure(MEASURE::DISPARITY)).copyTo(SbSResult[x](cv::Rect(width, 0, width, height)));
//            // slMat2cvMat(zed[x]->retrieveMeasure_gpu(XYZRGBA)).copyTo(BufferXYZRGBA[x]);
//            slMat2cvMat(zed[x]->retrieveMeasure(XYZRGBA)).copyTo(BufferXYZRGBA[x]);
//        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    }
//    delete zed[x];
//}

//void AugmentedVR::retrieve_ZED_PC_CPU(){
//#ifdef EVAL
//    timeval start,end;
//    gettimeofday(&start, NULL);
//    cout << ">>>>>>> PC thread starting at: " << double(start.tv_sec)*1000 + double(start.tv_usec) / 1000<< "ms" << endl;
//#endif
//    slMat2cvMat(mZEDCam->retrieveMeasure(XYZRGBA)).copyTo(pointcloud);
//
//    if (!INIT_FLAG) {
//        pointcloud.copyTo(initPC);
//        INIT_FLAG = true;
//    }
//    cv::Mat PCChannels[3];
//
//    for (int i=0;i<3;i++){
//        cv::extractChannel(pointcloud,PCChannels[i],i);
//    }
//
//    merge(PCChannels,3,PC_noColor);
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << ">>>>>>> PC thread ending at: " << double(end.tv_sec)*1000 + double(end.tv_usec) / 1000<< "ms";
//    cout << "(" << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms)" << endl;
//#endif
//    return;
//}


bool AugmentedVR::load_NextFrame() {
    if (SlamFrame.FrameLeft.empty()) return false;
    SlamFrame.pointcloud.copyTo(pointcloud);
    SlamFrame.PC_noColor.copyTo(PC_noColor);
    SlamFrame.FrameLeft.copyTo(FrameLeft);
    SlamFrame.FrameRight.copyTo(FrameRight);
    SlamFrame.FrameLeftGray.copyTo(FrameLeftGray);
    SlamFrame.FrameRightGray.copyTo(FrameRightGray);
    ZEDTS = SlamFrame.ZEDTS;
    frameTS = SlamFrame.frameTS;
    SlamFrame = ZED_DATA{};
    return true;
}

void AugmentedVR::PrepareNextFrame() {
    // move current next frame to slam buffer
    SlamFrame = NextFrame;
    NextFrame = ZED_DATA{};


#ifdef PIPELINE
    thread prefetch(&AugmentedVR::grabNextZEDFrameOffline,this);
#else
    grabNextZEDFrameOffline();
#endif

    // create next frame for slam as well...
    // use gray to save conversion time.
    // thread feedslam(&AugmentedVR::FeedSlamNextFrame,this);
    FeedSlamNextFrame();

#ifdef PIPELINE
    prefetch.join();
#endif
}

bool AugmentedVR::grabNextZEDFrameOffline() {

#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
//    cout << "grabZEDFrameOffline >>>>>>> PC thread starting at: " << double(start.tv_sec)*1000 + double(start.tv_usec) / 1000<< "ms" << endl;
#endif
    bool err = mZEDCam->grab(senseMode, 1, 1, 1);

    if (err) {
        cerr << "can't grab image" << endl;
        return false;
    }
    if (!err) {
        frameSeq++;
        NextFrame.ZEDTS = mZEDCam->getCameraTimestamp();
        NextFrame.frameTS = NextFrame.ZEDTS - startTS;
#ifdef EVAL
        gettimeofday(&end, NULL);
        cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "grabZEDFrameOffline >>>>>>>  Grab: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
        gettimeofday(&start, NULL);
#endif
        //sl::zed::Mat depthMM = zed[x]->retrieveMeasure(MEASURE::DEPTH);
//        slMat2cvMat(mZEDCam->retrieveImage(SIDE::LEFT)).copyTo(SbSResult(cv::Rect(0, 0, width, height)));
//        slMat2cvMat(mZEDCam->normalizeMeasure(MEASURE::DEPTH)).copyTo(SbSResult(cv::Rect(width, 0, width, height)));
//        sl::zed::Mat slmat =  mZEDCam->retrieveMeasure(XYZ);

//        sl::zed::Mat slmat =  mZEDCam->retrieveMeasure_gpu(XYZRGBA);
//#ifdef EVAL
//        gettimeofday(&end, NULL);
//        cout << "   PC in GPU: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//        gettimeofday(&start, NULL);
//#endif
        slMat2cvMat(mZEDCam->retrieveMeasure(XYZRGBA)).copyTo(NextFrame.pointcloud);

        if (!INIT_FLAG) {
            NextFrame.pointcloud.copyTo(initPC);
            INIT_FLAG = true;
        }
        cv::Mat PCChannels[3];

        for (int i=0;i<3;i++){
            cv::extractChannel(NextFrame.pointcloud,PCChannels[i],i);
        }

        merge(PCChannels,3,NextFrame.PC_noColor);
//
//
#ifdef EVAL
        gettimeofday(&end, NULL);
        cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "grabZEDFrameOffline >>>>>>>   PC in CPU: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
        gettimeofday(&start, NULL);
#endif

//        depth_mat = slMat2cvMat(mZEDCam->retrieveMeasure(MEASURE::DEPTH));
        NextFrame.FrameLeft = slMat2cvMat(mZEDCam->retrieveImage(SIDE::LEFT));
        NextFrame.FrameRight = slMat2cvMat(mZEDCam->retrieveImage(SIDE::RIGHT));

        cv::cvtColor(NextFrame.FrameLeft, NextFrame.FrameLeftGray, CV_BGR2GRAY);
        cv::cvtColor(NextFrame.FrameRight, NextFrame.FrameRightGray, CV_BGR2GRAY);
#ifdef EVAL
        gettimeofday(&end, NULL);
        cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "grabZEDFrameOffline >>>>>>> Frame: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//        cout << ">>>>>>> PC thread ending at: " << double(end.tv_sec)*1000 + double(end.tv_usec) / 1000<< "ms"<< endl;
#endif
//        BufferXYZRGBA_gpu = mZEDCam->retrieveMeasure_gpu(XYZRGBA);

        // cout << BufferXYZRGBA[x](cv::Rect(0,0,2,2));

//        cv::resize(SbSResult, ZED_LRes, DisplaySize);
    }


    return true;
}


bool AugmentedVR::calcOpticalFlow(){
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    cv::Size subPixWinSize(10,10), winSize(31,31);

    if (FRAME_ID%DUTYCYCLE==0){
        const int MAX_COUNT = 500;
        // the keypoints are updated! re-init
        keypoints = keypoints_cache;    status = status_cache;      err  = err_cache;
        keypoints_cache = vector<cv::Point2f>();  status_cache = vector<uchar>();   err_cache = vector<float>();

        cv::goodFeaturesToTrack(LastFrame.FrameLeftGray, keypoints_cache, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
        cornerSubPix(LastFrame.FrameLeftGray, keypoints_cache, subPixWinSize, cv::Size(-1,-1), termcrit);
    }
    cv::Mat lastFrameLeftGray, lastFrameLeft;
    LastFrame.FrameLeftGray.copyTo(lastFrameLeftGray);
    LastFrame.FrameLeft.copyTo(lastFrameLeft);

#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
#endif

#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "fetchNUpdateFrameNPointcloud >>>>> Fetch: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
    gettimeofday(&start, NULL);
#endif

    vector<cv::Point2f> lastKeyPoints = keypoints;  vector<cv::Point2f> lastKeyPoints_cache = keypoints_cache;
    keypoints = vector<cv::Point2f>();              keypoints_cache = vector<cv::Point2f>();

    if (lastFrameLeftGray.empty() || lastKeyPoints_cache.empty()) return true;

    // Double Cache: track new keypoints to cache for future use
    vector<uchar> new_status_cache;    vector<float> new_err_cache;
    // 2D track
    calcOpticalFlowPyrLK(lastFrameLeftGray, FrameLeftGray, lastKeyPoints_cache, keypoints_cache, new_status_cache, new_err_cache, winSize, 3, termcrit, 0, 0.001);

    if (status_cache.empty()){
        status_cache = new_status_cache;
    }else{
        for (int i=0;i<status_cache.size();i++){
            status_cache[i] &= new_status_cache[i];
        }
    }


    if (lastKeyPoints.empty()) return true;
    // Double Cache: track old keypoints to compare and filter

    vector<uchar> new_status;    vector<float> new_err;
    // 2D track
    calcOpticalFlowPyrLK(lastFrameLeftGray, FrameLeftGray, lastKeyPoints, keypoints, new_status, new_err, winSize, 3, termcrit, 0, 0.001);

    if (status.empty()){
        status = new_status;
    }else{
        for (int i=0;i<status.size();i++){
            status[i] &= new_status[i];
        }
    }
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "fetchNUpdateFrameNPointcloud >>>>>> OF: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
    gettimeofday(&start, NULL);
#endif
    // FIND HOMOGRAPHY MATRIX
    //-- Localize the object
    std::vector<cv::Point2f> good_lastKeypoints, good_Keypoints;
    std::vector<cv::Point2f> obj_corners(4);
    std::vector<cv::Point2f> scene_corners(4);

    for( size_t i = 0; i < keypoints_cache.size(); i++ ){
        if (status_cache[i]){
            good_lastKeypoints.push_back(lastKeyPoints_cache[i]);
            good_Keypoints.push_back(keypoints_cache[i]);
        }
    }
    cv::Mat H;
    if (!good_Keypoints.empty())
        H = findHomography( good_Keypoints,good_lastKeypoints, cv::RANSAC );
    //-- Get the corners from the image_1 ( the object to be "detected" )
    int margin=50;
    obj_corners[0] = cvPoint(margin,margin); obj_corners[1] = cvPoint( FrameLeft.cols-margin, margin );
    obj_corners[2] = cvPoint( FrameLeft.cols-margin, FrameLeft.rows-margin ); obj_corners[3] = cvPoint( margin, FrameLeft.rows-margin );

    H.copyTo(sceneTransformMat);
//    cout << "Homography matrix: " << H << endl
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "fetchNUpdateFrameNPointcloud>>>>>>Homography: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
#endif
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    // debug
    if (DEBUG && VISUAL){
        cv::Mat img, img_cache;
        FrameLeft.copyTo(img);lastFrameLeft.copyTo(img_cache);
        int count = 0;

//        for( size_t i = 0; i < keypoints.size(); i++ ){
//            if (status[i]){
//                count ++;
//                circle( img, lastKeyPoints[i], 3, Scalar(255,0,0), -1, 8);
//                circle( img, keypoints[i], 3, Scalar(0,255,0), -1, 8);
//                line( img, lastKeyPoints[i], keypoints[i], Scalar(0,0,255));
//            }
//        }
//        imshow("Current Tracking Frame", img);
//        cout << "tracking " << count << " keypoints\n";


        for( size_t i = 0; i < keypoints_cache.size(); i++ ){
            if (status_cache[i]){
                count ++;
                circle( img_cache, lastKeyPoints_cache[i], 3, cv::Scalar(255,0,0), -1, 8);
                circle( img_cache, keypoints_cache[i], 3, cv::Scalar(0,255,0), -1, 8);
                line( img_cache, lastKeyPoints_cache[i], keypoints_cache[i], cv::Scalar(0,0,255));
            }
        }
        if (!H.empty()){

            perspectiveTransform( obj_corners, scene_corners, H);
            line( img_cache, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 4 );
            line( img_cache, scene_corners[1], scene_corners[2], cv::Scalar( 0, 255, 0), 4 );
            line( img_cache, scene_corners[2], scene_corners[3], cv::Scalar( 0, 255, 0), 4 );
            line( img_cache, scene_corners[3], scene_corners[0], cv::Scalar( 0, 255, 0), 4 );
        }
        imshow("Cache Tracking in Last Frame", img_cache);
//        cout << "tracking " << count << " keypoints\n";
    }

    return true;
}


//void AugmentedVR::fetchNUpdateFrameNPointcloud(){
//
//    if (frameSeq%5==0) // TODO: to be removed, only for debug now
////        for (int i = 0; i < NUM_CAMERAS; i++) {
//            FrameLeft.copyTo(lastFrameLeft);
//            FrameRight.copyTo(lastFrameRight);
//            pointcloud.copyTo(lastpointcloud);
//            CamMotionMat.copyTo(lastCamMotionMat);
////        }
//    grabZEDFrameOffline();
//
//    for (int i = 0; i < NUM_CAMERAS; i++) {
//        if (OFFLINE){
//            while(frame_ts[i] - frame_ts[1-i] < 0){
//                grab_offline(i);
//                frame_ts[i] = ZED_Timestamp[i] - start_timestamp[i];
//            }
//            // cout << "skipping camera 1 frame ts: " << ZED_Timestamp[0] << endl;
//        }
//
//        char wnd_name[21];
//        sprintf(wnd_name, "ZED N° %d", i);
//        cv::resize(SbSResult[i], ZED_LRes[i], DisplaySize);
//
//        if (SHOW_IMG) cv::imshow(wnd_name, ZED_LRes[i]);
//        // cout << "copying pont cloud\n";
////            if (SHOW_PC)
////                for (int i = 0; i < NUM_CAMERAS; i++)
////                    BufferXYZRGBA[i].copyTo(pointcloud[i]);
//    }
//}

//void AugmentedVR::updateLastPointCloud(){
//    cv::Mat tmpPC;
//    pointcloud.copyTo(tmpPC);
//    lastpointcloud.push_back(tmpPC);
//    lastpointcloud.erase(lastpointcloud.begin());
//    //    pointcloud.copyTo(lastpointcloud);
//}


//bool AugmentedVR::DutyCycling(){
//    if (FRAME_ID%DUTYCYCLE==0) return true;
//    return false;
//}

void AugmentedVR::updateLastStereoData(){
    LastFrame = STEREO_DATA{};

    LastFrame.frameTS = frameTS;
    LastFrame.ZEDTS = ZEDTS;
    LastFrame.frameSeq = frameSeq;

    FrameLeft.copyTo(LastFrame.FrameLeft);
    FrameRight.copyTo(LastFrame.FrameRight);
    FrameLeftGray.copyTo(LastFrame.FrameLeftGray);
    FrameRightGray.copyTo(LastFrame.FrameRightGray);

    CamMotionMat.copyTo(LastFrame.CamMotionMat);


    pointcloud.copyTo(LastFrame.pointcloud);
    PC_noColor.copyTo(LastFrame.PC_noColor);
    sceneTransformMat.copyTo(LastFrame.sceneTransformMat);
//    stereoTmp.keypoints = keypoints;//TODO: probably don't need this
    LastFrame.keypoints = keypoints_cache;
    LastFrame.tracked_keypoints = keypoints;

    LastFrame.tracked_status = status;

    lastStereoData.push_back(LastFrame);
    lastStereoData.erase(lastStereoData.begin());

//    updateLastPointCloud();
}


// call SLAM and calculate camera pose relative to last frame
// update rotation and translation matrxi to last frame: Rlc, tlc
void AugmentedVR::trackCam() {
    CamMotionMat = mSLAM->TrackStereo(FrameLeft, FrameRight, frameTS);
    // for debug only
//    if(COOP){
//        CamMotionMat.at<float>(0,3) +=7;
//    }

    if (SHOW_CAMMOTION) cout << "CamMotionMat: \n" << CamMotionMat << endl;

}

void AugmentedVR::analyze(){

#ifdef EVAL
    timeval tTotalStart, tFetchStart, tCacheStart, tSlamStart, tPCMotionStart, tPCMotionFilterStart, tObjectFilterStart, tTXStart, tRXStart, tPCmergeStart,tDeadReckonStart;
    timeval tTotalEnd, tFetchEnd, tCacheEnd, tSlamEnd, tPCMotionEnd, tPCMotionFilterEnd, tObjectFilterEnd, tTXEnd, tRXEnd, tPCmergeEnd, tDeadReckonEnd;
#endif

    if (!lastStereoData[0].keypoints.empty()){

#ifdef EVAL
        gettimeofday(&tPCMotionStart, NULL);
#endif

//        if (FRAME_ID % DUTYCYCLE == DUTYCYCLE-1){

        calcPCMotionVec();
        //        VNode[1]->calcPCMotionVec();
#ifdef EVAL
        gettimeofday(&tPCMotionEnd, NULL);
        cout << "TimeStamp: " << double(tPCMotionEnd.tv_sec-tInit.tv_sec)*1000 + double(tPCMotionEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "analyze>>>>>>PC Motion: " << double(tPCMotionEnd.tv_sec-tPCMotionStart.tv_sec)*1000 + double(tPCMotionEnd.tv_usec-tPCMotionStart.tv_usec) / 1000<< "ms" << endl;
        gettimeofday(&tPCMotionFilterStart, NULL);
#endif

        filterPCMotion();
//        VNode[1]->filterPCMotion();
#ifdef EVAL
        gettimeofday(&tPCMotionFilterEnd, NULL);
        cout << "TimeStamp: " << double(tPCMotionFilterEnd.tv_sec-tInit.tv_sec)*1000 + double(tPCMotionFilterEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "analyze>>>>>>PC Motion Filter: " << double(tPCMotionFilterEnd.tv_sec-tPCMotionFilterStart.tv_sec)*1000 + double(tPCMotionFilterEnd.tv_usec-tPCMotionFilterStart.tv_usec) / 1000<< "ms" << endl;
        gettimeofday(&tObjectFilterStart, NULL);
#endif

//            VNode[0]->PC_segmentation(ZEDCACHESIZE-1);

        filterObjectMotion(ZEDCACHESIZE-1);
//        }

#ifdef EVAL
        gettimeofday(&tObjectFilterEnd, NULL);
        cout << "TimeStamp: " << double(tObjectFilterEnd.tv_sec-tInit.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "analyze>>>>>>PC Object Filter: " << double(tObjectFilterEnd.tv_sec-tObjectFilterStart.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tObjectFilterStart.tv_usec) / 1000<< "ms"<< endl;
        cout << "TimeStamp: " << double(tObjectFilterEnd.tv_sec-tInit.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "analyze: " << double(tObjectFilterEnd.tv_sec-tPCMotionStart.tv_sec)*1000 + double(tObjectFilterEnd.tv_usec-tPCMotionStart.tv_usec) / 1000<< "ms"<< endl;
#endif

        // all computation done, prep fro next frame
    }

}

void AugmentedVR::calcPCMotionVec() {

//    for (int i=0;i<lastStereoData.size();i++){
    int i = 0; // choose the oldest and cacl the displacement
        if (!lastStereoData[i].CamMotionMat.empty()){

            calcCamMotion(i);

            transfromPCtoLastFrameCoord(i);

            lastStereoData[i].PCMotionVec = transformedPointcloud - lastStereoData[i].pointcloud;
        }
//    }
}



//
//// matching keypoints from last frame, update perspective transformation matrix to get PC one by one corespondance
//void AugmentedVR::calcSceneTransformMat(int idx) {
//    int margin = 50;
//    cv::Mat img_cur_scene = getGrayBBox(FrameLeft, 0, 0, width - 2 * margin,
//                                        height - 2 * margin); //find homography func cannot find the full scene.
//    cv::Mat img_last_scene = getGrayBBox(lastStereoData[idx].FrameLeft, 0, 0, width, height);
////    landmark_matchinfo matched_scene = find_obj_in_second_scene(img_cur_scene,
////                                                                img_last_scene); // current scene in last scene, because last scene is larger
//    lastStereoData[idx].matched_scene = find_obj_in_second_scene(img_cur_scene,
//                                                                img_last_scene); // current scene in last scene, because last scene is larger
//
//    lastStereoData[idx].matched_scene.perspectiveTransformMatrix.copyTo(sceneTransformMat);
//
//    if (DEBUG > 1) {
//        cv::Size outputSize(width, height);
////            imwrite("current scene.jpg",img_cur_scene);
////            imwrite("last scene.jpg", img_last_scene);
//        imshow("current scene.jpg", img_cur_scene);
//        imshow("last scene.jpg", img_last_scene);
//        cv::Mat transformFrame;
//        cv::warpPerspective(FrameLeft, transformFrame, sceneTransformMat, outputSize);
//        imshow("transformed current scene", transformFrame);
//    }
//
//}




void AugmentedVR::calcCamMotion(int idx){
    // w: world, l: last frame, c: current frame
    if (lastStereoData[idx].CamMotionMat.empty() || LastFrame.CamMotionMat.empty()) return;
    const cv::Mat Rcw = LastFrame.CamMotionMat.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = LastFrame.CamMotionMat.rowRange(0,3).col(3);
    const cv::Mat Rwc = Rcw.t();
    const cv::Mat twc = -Rwc*tcw;

    const cv::Mat Rlw = lastStereoData[idx].CamMotionMat.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = lastStereoData[idx].CamMotionMat.rowRange(0,3).col(3);

    tlc = Rlw*twc+tlw;
    Rlc = Rlw*Rwc;
    if (DEBUG>1){
        cout << "tlc: \n" << tlc << endl;
        cout << "Rlc: \n" << Rlc << endl; // to see if Rlc is almost identity matrix
    }
}
void AugmentedVR::transfromPCtoLastFrameCoord(int idx){
    // perspective transform to get pixel level pointcloud matching from last frame
    cv::Size outputSize(width, height);

    timeval start, end;
    gettimeofday(&start, NULL);
    cv::warpPerspective(LastFrame.pointcloud,transformedPointcloud,LastFrame.sceneTransformMat,outputSize);
    gettimeofday(&end, NULL);
    //cout << "   CPU warpperspective: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;

//    cv::cuda::GpuMat tmp(pointcloud);
//    cv::cuda::warpPerspective(pointcloud,transformedPointcloud,sceneTransformMat,outputSize);


    if (!lastStereoData[idx].CamMotionMat.empty()){ //&& FRAME_ID>2
//            cv::Mat transformedPC_Coords(height, width, CV_32FC3);
//        int from_to[] = { 0,0, 1,1, 2,2 };
//            cv::mixChannels(transformedPC, transformedPC_Coords, from_to, 3);


        // TODO: speed up the operations
//            for (int x=0;x<transformedPC_Coords.cols;x++){
//                for (int y=0;y<transformedPC_Coords.rows;y++){
//                    transformedPC_Coords_inLast(cv::Rect(x,y,1,1)) = Rlc * transformedPC_Coords(cv::Rect(x,y,1,1))+tlc;
//                }
//            }

        // TODO: use tlc only for now
//            cout << "tlc(1,0): " << tlc.at<float>(1,0) << endl;
//            cout << "tlc mat type: " << tlc.type() << endl;
//            cv::Mat tlc_mat(height, width, CV_32FC3, tlc_vec);
//            cout << "tlc: " << tlc_mat(cv::Rect(dbx,dby,1,1)) << endl;
//            cv::Mat tlc_chan[4];
//            cv::split(tlc_mat,tlc_chan);
//            tlc_chan[3]=cv::Mat(height,width,CV_32F,);
//            cv::Mat tlc_mat_4;
//            cv::merge(tlc_chan,4,tlc_mat_4);

        // watch out, orbslam and zed have different default coordinate frame positive difrection
        // same x, opposite y,z

        cv::Scalar tlc_vec = cv::Scalar(tlc.at<float>(0,0),-tlc.at<float>(1,0),-tlc.at<float>(2,0),0);
        cv::Mat tlc_mat(height, width, CV_32FC4, tlc_vec);
        transformedPointcloud = transformedPointcloud+tlc_mat;

        if (DEBUG>1){
            cout << "tlc: " << tlc_mat(cv::Rect(dbx,dby,1,1)) << endl;
//                cout << "tlc_mat4: " << tlc_mat_4(cv::Rect(dbx,dby,1,1)) << endl;
            cout << "transformedPC_inLast: " << transformedPointcloud(cv::Rect(dbx,dby,1,1)) << endl;
        }

    }
}


//Return relative transformation matrix from received frame to current frame
cv::Mat AugmentedVR::calcRelaCamPos(cv::Mat TcwReceived){
    // w: world, r: received frame, c: current frame
//    if (TcwReceived.empty()) return cv::Mat;
    const cv::Mat Rcw = LastFrame.CamMotionMat.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = LastFrame.CamMotionMat.rowRange(0,3).col(3);
    const cv::Mat Rwc = Rcw.t();
    const cv::Mat twc = -Rwc*tcw;

    const cv::Mat Rrw = TcwReceived.rowRange(0,3).colRange(0,3);
    const cv::Mat trw = TcwReceived.rowRange(0,3).col(3);

    cv::Mat trc = Rrw*twc+trw;
    cv::Mat Rrc = Rrw*Rwc;

    cv::Mat Trc = cv::Mat::eye(4,4,CV_32F);
    trc.copyTo(Trc.rowRange(0,3).col(3));
    Rrc.copyTo(Trc.rowRange(0,3).colRange(0,3));
    if (DEBUG){
        cout << "trc: \n" << trc << endl;
        cout << "Rrc: \n" << Rrc << endl; // to see if Rlc is almost identity matrix
    }
//    return trc; // only trc for now, TODO: include Rrc after you can compute perelement 1 by 3 multiplication
    return Trc;
}


cv::Mat AugmentedVR::transfromRxPCtoMyFrameCoord(cv::Mat trc, cv::Mat PCReceived){

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

cv::Mat AugmentedVR::getGrayBBox(cv::Mat img, int select_left, int select_top, int select_width, int select_height){
    // legalize
    select_left = select_left > 0 ? select_left : 0;
    select_top = select_top > 0 ? select_top : 0;
    select_width = select_width+select_left < img.cols ? select_width : img.cols - select_left;
    select_height = select_height+select_top < img.rows ? select_height : img.rows - select_top;

    cv::Mat img_tmp = img(cv::Rect(select_left, select_top, select_width, select_height));

    cv::cvtColor(img_tmp, img_tmp, CV_BGR2GRAY);
    return img_tmp;
//    imwrite("img_tmp.jpg",img_tmp);
//    return imread( "img_tmp.jpg", IMREAD_GRAYSCALE );
}

//// TODO: move to utils
//landmark_matchinfo AugmentedVR::find_obj_in_second_scene(cv::Mat img_object, cv::Mat img_scene)
//{
//    //-- Localize the object
//    std::vector<Point2f> obj;
//    std::vector<Point2f> scene;
//    std::vector<Point2f> obj_corners(4);
//    std::vector<Point2f> scene_corners(4);
//
//
//
//
//#ifdef EVAL
//    timeval start, end;
//    gettimeofday(&start, NULL);
//#endif
//
////    if( !img_object.data || !img_scene.data )
////  { std::cout<< " --(!) Error reading images " << std::endl; return NULL; }
//    //-- Step 1: Detect the keypoints and extract descriptors using SURF
//    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create( minHessian );
//
//    // SurfFeatureDetector detector( minHessian );
//    std::vector<KeyPoint> keypoints_object, keypoints_scene;
//    cv::Mat descriptors_object, descriptors_scene;
//    detector->detectAndCompute( img_object, cv::Mat(), keypoints_object, descriptors_object );
//    detector->detectAndCompute( img_scene, cv::Mat(), keypoints_scene, descriptors_scene );
//
//
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << "       feature detect: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//    gettimeofday(&start, NULL);
//#endif
//    //-- Step 2: Matching descriptor vectors using FLANN matcher
//    FlannBasedMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_object, descriptors_scene, matches );
//    double max_dist = 0; double min_dist = 100;
//    //-- Quick calculation of max and min distances between keypoints
//    if(DEBUG>1){
//        for( int i = 0; i < descriptors_object.rows; i++ )
//        { double dist = matches[i].distance;
//            if( dist < min_dist ) min_dist = dist;
//            if( dist > max_dist ) max_dist = dist;
//        }
//        printf("-- Max dist : %f \n", max_dist );
//        printf("-- Min dist : %f \n", min_dist );
//    }
//    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< DMatch > good_matches;
//    for( int i = 0; i < descriptors_object.rows; i++ )
//    { if( matches[i].distance < 3*min_dist )
//        { good_matches.push_back( matches[i]); }
//    }
//    cv::Mat img_matches;
//    if (VISUAL)
//        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
//                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                     std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//    for( size_t i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//    }
//
//
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << "       Match: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//    gettimeofday(&start, NULL);
//#endif
//
//    cv::Mat H = findHomography( obj, scene, RANSAC );
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
//    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
//
//    if (!H.empty())
//        perspectiveTransform( obj_corners, scene_corners, H);
//
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << "       HOMO and transform: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
////    gettimeofday(&start, NULL);
//#endif
////    cout << "Homography matrix: " << H << endl
//
//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    if (VISUAL){
//
//        line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
//        line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        //-- Show detected matches
//        if (DEBUG>1) imshow( "Good Matches & Object detection", img_matches );
//    }
//
//    int select_width,select_height,select_left, select_top;
//    if (!scene_corners.empty()){
//        int sum_x = 0; int sum_y = 0; int sum_w = 0; int sum_h = 0;
//        for (int ii=0;ii<4;ii++){
//            sum_x += scene_corners[ii].x;
//            sum_y += scene_corners[ii].y;
////            sum_w += abs(scene_corners[ii].x - scene_corners[(ii-1) % 4].x);
////            sum_x += scene_corners[ii].x;
////            sum_y += scene_corners[ii++].y;
////            sum_h += abs(scene_corners[ii].y - scene_corners[(ii-1) % 4].y);
//        }
//        sum_w = scene_corners[1].x - scene_corners[0].x + scene_corners[2].x - scene_corners[3].x;
//        sum_h = scene_corners[2].y - scene_corners[1].y + scene_corners[3].y - scene_corners[0].y;
//        select_left = sum_x / 4 - sum_w / 4;
//        select_top = sum_y / 4 - sum_h / 4;
//        select_width = sum_w / 2; select_height = sum_h / 2;
//    }
//    landmark_matchinfo ret(img_object, H, select_left,select_top,select_width, select_height,obj,scene);
//    return ret;
////  return scene_corners;
//}

//landmark_matchinfo AugmentedVR::find_obj_in_second_scene(cv::Mat img_object, cv::Mat img_scene)
//{
//
//    // cvtColor(img_object, img_object, cv::COLOR_RGB2GRAY);
//    // cvtColor(img_scene, img_scene, cv::COLOR_RGB2GRAY);
//
//    // Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
//    // Mat img_scene = imread( argv[2], IMREAD_GRAYSCALE );
//
//    //-- Localize the object
//    std::vector<Point2f> obj;
//    std::vector<Point2f> scene;
//    std::vector<Point2f> obj_corners(4);
//    std::vector<Point2f> scene_corners(4);
//
//
//
//
//#ifdef EVAL
//    timeval start, end;
//    gettimeofday(&start, NULL);
//#endif
//
////    if( !img_object.data || !img_scene.data )
////  { std::cout<< " --(!) Error reading images " << std::endl; return NULL; }
//    //-- Step 1: Detect the keypoints and extract descriptors using SURF
//    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create( minHessian );
//
//    // SurfFeatureDetector detector( minHessian );
//    std::vector<KeyPoint> keypoints_object, keypoints_scene;
//    cv::Mat descriptors_object, descriptors_scene;
//    detector->detectAndCompute( img_object, cv::Mat(), keypoints_object, descriptors_object );
//    detector->detectAndCompute( img_scene, cv::Mat(), keypoints_scene, descriptors_scene );
//
//
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << "       feature detect: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//    gettimeofday(&start, NULL);
//#endif
//    //-- Step 2: Matching descriptor vectors using FLANN matcher
//    FlannBasedMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_object, descriptors_scene, matches );
//    double max_dist = 0; double min_dist = 100;
//    //-- Quick calculation of max and min distances between keypoints
//    if(DEBUG>1){
//        for( int i = 0; i < descriptors_object.rows; i++ )
//        { double dist = matches[i].distance;
//            if( dist < min_dist ) min_dist = dist;
//            if( dist > max_dist ) max_dist = dist;
//        }
//        printf("-- Max dist : %f \n", max_dist );
//        printf("-- Min dist : %f \n", min_dist );
//    }
//    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< DMatch > good_matches;
//    for( int i = 0; i < descriptors_object.rows; i++ )
//    { if( matches[i].distance < 3*min_dist )
//        { good_matches.push_back( matches[i]); }
//    }
//    cv::Mat img_matches;
//    if (VISUAL)
//        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
//                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                     std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//    for( size_t i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//    }
//
//
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << "       Match: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//    gettimeofday(&start, NULL);
//#endif
//
//    cv::Mat H = findHomography( obj, scene, RANSAC );
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
//    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
//
//    if (!H.empty())
//        perspectiveTransform( obj_corners, scene_corners, H);
//
//#ifdef EVAL
//    gettimeofday(&end, NULL);
//    cout << "       HOMO and transform: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
////    gettimeofday(&start, NULL);
//#endif
////    cout << "Homography matrix: " << H << endl
//
//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    if (VISUAL){
//
//        line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
//        line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//        //-- Show detected matches
//        if (DEBUG>1) imshow( "Good Matches & Object detection", img_matches );
//    }
//
//
//    // TODO:try to find those matches that doesn't align with H
////    cv::Mat img; FrameLeft.copyTo(img);
////    std::vector<Point2f> obj_pts;
////    std::vector<Point2f> obj_pts_trans;
////    std::vector<Point2f> scene_pts;
////    for( size_t i = 0; i < matches.size(); i++ )
////    {
////        if( matches[i].distance < 3*min_dist ){
////
////            obj_pts.push_back( keypoints_object[ matches[i].queryIdx ].pt );
////            scene_pts.push_back( keypoints_scene[ matches[i].trainIdx ].pt );
////        }
////    }
////    if (!H.empty())
////        perspectiveTransform( obj_pts, obj_pts_trans, H);
////    for( size_t i = 0; i < obj_pts_trans.size(); i++ )
////    {
////        if (norm(obj_pts_trans[i]-scene_pts[i]) > 2){
////            circle( img, obj_pts_trans[i], 3, Scalar(255,0,0), -1, 8);
////            circle( img, scene_pts[i], 3, Scalar(0,255,0), -1, 8);
////            line( img, obj_pts_trans[i], scene_pts[i], Scalar(0,0,255));
////        }
////    }
////    imshow ("moving matches", img);
//
//    /// TODO: trying klt on surf features
////    cv::Mat img; FrameLeft.copyTo(img);
////    std::vector<Point2f> obj_pts, obj_pts_tracked;
////    std::vector<Point2f> obj_pts_trans;
////
////    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
////    Size subPixWinSize(10,10), winSize(31,31);
////
//////    cornerSubPix(img_object, obj_pts, subPixWinSize, Size(-1,-1), termcrit);
////
////    vector<uchar> status;
////    vector<float> err;
////
////    calcOpticalFlowPyrLK(img_object, img_scene, obj_pts, obj_pts_tracked, status, err, winSize,
////                         3, termcrit, 0, 0.001);
////    if (!H.empty())
////        perspectiveTransform( obj_pts, obj_pts_trans, H);
////    for( size_t i = 0; i < obj_pts_tracked.size(); i++ )
////    {
////        if (status[i] && norm(obj_pts_trans[i]-obj_pts_tracked[i]) > 2){
////            circle( img, obj_pts_trans[i], 3, Scalar(255,0,0), -1, 8);
////            circle( img, obj_pts_tracked[i], 3, Scalar(0,255,0), -1, 8);
////            line( img, obj_pts_trans[i], obj_pts_tracked[i], Scalar(0,0,255));
////        }
////    }
////    imshow("KLT matches", img);
//
//
////    if (DEBUG) imwrite("./match.jpg", img_matches);
//    // waitKey(0);
//    int select_width,select_height,select_left, select_top;
//    if (!scene_corners.empty()){
//        int sum_x = 0; int sum_y = 0; int sum_w = 0; int sum_h = 0;
//        for (int ii=0;ii<4;ii++){
//            sum_x += scene_corners[ii].x;
//            sum_y += scene_corners[ii].y;
////            sum_w += abs(scene_corners[ii].x - scene_corners[(ii-1) % 4].x);
////            sum_x += scene_corners[ii].x;
////            sum_y += scene_corners[ii++].y;
////            sum_h += abs(scene_corners[ii].y - scene_corners[(ii-1) % 4].y);
//        }
//        sum_w = scene_corners[1].x - scene_corners[0].x + scene_corners[2].x - scene_corners[3].x;
//        sum_h = scene_corners[2].y - scene_corners[1].y + scene_corners[3].y - scene_corners[0].y;
//        select_left = sum_x / 4 - sum_w / 4;
//        select_top = sum_y / 4 - sum_h / 4;
//        select_width = sum_w / 2; select_height = sum_h / 2;
//    }
//    landmark_matchinfo ret(img_object, H, select_left,select_top,select_width, select_height,obj,scene);
//    return ret;
////  return scene_corners;
//}


cv::Mat AugmentedVR::calcPCDisplacement(int idx){
    if (lastStereoData[ZEDCACHESIZE-idx].PCMotionVec.empty()){
        return cv::Mat(height,width, CV_32F, cv::Scalar(0.));
    }
    cv::Mat PCChannels[3];
    cv::Mat PCChan2[3];
    cv::Mat PCDisplacement(height,width, CV_32F, cv::Scalar(0.));
//        cv::Mat PCDisplacement(2,2, CV_32F, Scalar(0.));

    for (int i=0;i<3;i++){
        cv::extractChannel(lastStereoData[ZEDCACHESIZE-idx].PCMotionVec,PCChannels[i],i);
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

void onMouseCallback_DisplayDisplacement(int32_t event, int32_t x, int32_t y, int32_t flag, void* param) {

    cv::Mat* PC_dist =(cv::Mat*)param;
    if (event == cv::EVENT_LBUTTONDOWN) {
        cout << "Point: (" << x << "," << y << "), PC: " << (*PC_dist)(cv::Rect(x,y,1,1)) << ", Thresh: "<< MOTIONTHRESH_PERPIXEL*ZEDCACHESIZE <<endl;
    }
}


void AugmentedVR::filterPCMotion() { // idx: 0, 1frame,  1, 2frames, 2  3 frames back ...etc
    int idx = ZEDCACHESIZE-1; // number of frames back [0, CACHESIZE-1]
//    for (int idx=0;idx<ZEDCACHESIZE;idx++){
    if (lastStereoData[ZEDCACHESIZE-idx-1].CamMotionMat.empty())  return;

//        cv::Mat PCDisplacement;
//    cv::Mat PCDisplacement_15,PCDisplacement_10;
//    PCDisplacement_15 = calcPCDisplacement(3);
//    imshow("15 frames", PCDisplacement_15);
//    PCDisplacement_10 = calcPCDisplacement(2);
//    imshow("10 frames", PCDisplacement_10);
    PCDisplacement = calcPCDisplacement(idx+1);
    if (DEBUG && VISUAL){

        char tmp[50];
        sprintf(tmp, "Displacement", (idx+1)*5);
        imshow(tmp, PCDisplacement);
        cv::setMouseCallback(tmp, onMouseCallback_DisplayDisplacement, &PCDisplacement);
    }


    //TODO: abstract to function
    // THRESHOLDING TO GET THE MASK OF DYNAMIC OBJECTS
//        cv::Mat MotionMask, BoundaryNoiseMask;
    float motionThreshold = MOTIONTHRESH_PERPIXEL*ZEDCACHESIZE*(ZEDCACHESIZE-idx); //in meters (0.1-0.15 per frame )
//        cout << motionThreshold << endl;
//    timeval start, upload, thresh,download;
//    gettimeofday(&start, NULL);
    cv::threshold( PCDisplacement, lastStereoData[ZEDCACHESIZE-idx-1].MotionMask, motionThreshold, 255, cv::THRESH_BINARY);
//    gettimeofday(&thresh, NULL);
//    cout << "CPU: thresholding: " << double(thresh.tv_sec-start.tv_sec)*1000 + double(thresh.tv_usec-start.tv_usec) / 1000 << "ms" << endl;

//    cv::cuda::GpuMat PCdis;
//    cv::Mat tmp;
//
//
//    gettimeofday(&start, NULL);
//    PCdis.upload(PCDisplacement);
//    gettimeofday(&upload, NULL);
//    cv::cuda::threshold(PCdis, PCdis, motionThreshold, 255.0, THRESH_BINARY);
//    gettimeofday(&thresh, NULL);
//    PCdis.download(tmp);
//    gettimeofday(&download, NULL);

//    cout << "GPU upload: " << double(upload.tv_sec-start.tv_sec)*1000 + double(upload.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
//    cout << "GPU thresholding: " << double(thresh.tv_sec-upload.tv_sec)*1000 + double(thresh.tv_usec-upload.tv_usec) / 1000 << "ms" << endl;
//    cout << "GPU download : " << double(download.tv_sec-thresh.tv_sec)*1000 + double(download.tv_usec-thresh.tv_usec) / 1000 << "ms" << endl;


//    float boundaryThreshold = 10; // to filter out noise
//    cv::threshold( PCDisplacement, BoundaryNoiseMask, boundaryThreshold, 255, THRESH_BINARY_INV);
//    cv::bitwise_and(MotionMask,BoundaryNoiseMask,MotionMask);

    if (DEBUG>1) cout << "MotionMask: " << lastStereoData[ZEDCACHESIZE-idx-1].MotionMask << endl;

//        pointcloud[0].copyTo(DynamicPC,MotionMask);
    lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.convertTo(lastStereoData[ZEDCACHESIZE-idx-1].MotionMask,CV_8U);
    if (DEBUG>1) cout << lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.depth() << lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.channels();
//        PCDisplacement.copyTo(DynamicPC,MotionMask);
//        cout << "Dynamic PC: " << DynamicPC << endl;
//    DynamicPC = Scalar(0,0,0,0); // reset value;
    lastStereoData[ZEDCACHESIZE-idx-1].pointcloud.copyTo(lastStereoData[ZEDCACHESIZE-idx-1].DynamicPC,lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
    lastStereoData[ZEDCACHESIZE-idx-1].FrameLeft.copyTo(lastStereoData[ZEDCACHESIZE-idx-1].DynamicFrame, lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);


    if (DEBUG>1){
        cv::Mat tmpPC;
        PCDisplacement.copyTo(tmpPC,lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
        cout << "filtered PCDisplacement: \n" << tmpPC << endl;
    }

//    if (DEBUG>1) {
//        cout << "PCDisplacement: " << PCDisplacement(cv::Rect(dbx,dby,1,1)) << endl;
//        cout << "lastPC: " << lastStereoData[ZEDCACHESIZE-1].pointcloud(cv::Rect(dbx,dby,1,1)) << endl;
//        cout << "curPC: " << LastFrame.pointcloud(cv::Rect(dbx,dby,1,1)) << endl;
////        cout << "trans_curPC: " << transformedPC(cv::Rect(dbx,dby,1,1)) << endl;
//        // TODO reenalle if needed for debug
////        cout << "transformedPC_inLast: " << transformedPointcloud(cv::Rect(dbx,dby,1,1)) << endl;
//        shiftPC(lastStereoData[ZEDCACHESIZE-idx-1].DynamicPC(cv::Rect(dbx,dby,dbw,dbh)),cv::Scalar(3,0,0,0));
//        shiftPC(transformedPointcloud(cv::Rect(dbx,dby,dbw,dbh)),cv::Scalar(3,0,0,0));
//    }
//    }
    //    Remove_HighLow
    cv::Mat tmpMask,tmpPC;
//    lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.copyTo(tmpMask);
    extractChannel(lastStereoData[ZEDCACHESIZE-idx-1].pointcloud,tmpPC,1);

    cv::threshold( tmpPC, tmpMask, HEIGHT_THRESH, 255, cv::THRESH_BINARY_INV);
    tmpMask.convertTo(tmpMask,CV_8U);
    lastStereoData[ZEDCACHESIZE-idx-1].MotionMask &= tmpMask;


}


void AugmentedVR::Remove_HighLow(int idx){
    // for now just remove single points... too many...*[]:
    // TODO: try to make the scane combined with check connection function
    cv::Mat tmpMask,tmpPC;
    int count = 0;



    // i is y, j is x
    for (int i=0;i<FrameLeft.rows;i++){
        for (int j=0;j<FrameLeft.cols;j++){
            if ((int)tmpMask.at<uchar>(i,j) == 255){


                cout << lastStereoData[ZEDCACHESIZE-idx-1].pointcloud(cv::Rect(j,i,1,1)) << endl;
                cout << tmpPC(cv::Rect(j,i,1,1)) << endl;
                cout << tmpPC.at<float>(i,j) << endl;

                if (tmpPC.at<double>(i,j) > HEIGHT_THRESH){
                    tmpMask.at<uchar>(i,j) = 0; continue;
                }

                if (i>0)
                    if ((int)tmpMask.at<uchar>(i-1,j)==255) continue;
                if (i<FrameLeft.cols-1)
                    if ((int)tmpMask.at<uchar>(i+1,j)==255) continue;
                if (j>0)
                    if ((int)tmpMask.at<uchar>(i,j-1)==255) continue;
                if (j<FrameLeft.rows-1)
                    if ((int)tmpMask.at<uchar>(i,j+1)==255) continue;

                tmpMask.at<uchar>(i,j) = 0;
                count ++;
            }
        }
    }
    cout << " filterd " << count << " solo points\n";
}


void AugmentedVR::CheckConnection(int tgt_x, int tgt_y, int cur_x, int cur_y, int idx, std::queue<cv::Point2f> & Q, cv::Mat & checkFlag,cv::Mat & inQueue, float motionThreshold){



    cv::Mat lastPC, curPC;
    lastStereoData[ZEDCACHESIZE-idx-1].PC_noColor.copyTo(lastPC);
    PC_noColor.copyTo(curPC);
    if (DEBUG) cout << "    Checking " << tgt_x << "," << tgt_y << ": " << (int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(tgt_y, tgt_x);
    if ((int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(tgt_y, tgt_x) == 255) {
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
        cout << (int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(cur_y, cur_x) << endl;
        if ((int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(cur_y, cur_x) ==0){
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

                lastStereoData[ZEDCACHESIZE-idx-1].MotionMask(cv::Rect(cv::Point2f(cur_x, cur_y), cv::Point2f(tgt_x, tgt_y)))   = 255;
                lastStereoData[ZEDCACHESIZE-idx-1].MotionMask(cv::Rect(cur_x, cur_y,1,1))   = 255;
                // recheck the connections
                if (cur_x>PATCHSIZE && cur_y>PATCHSIZE)                    CheckConnection(cur_x-PATCHSIZE, cur_y-PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue, motionThreshold);
                if (cur_x<FrameLeft.cols-PATCHSIZE && cur_y>PATCHSIZE)     CheckConnection(cur_x+PATCHSIZE, cur_y-PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
                if (cur_x>PATCHSIZE && cur_y<FrameLeft.rows-PATCHSIZE)                    CheckConnection(cur_x-PATCHSIZE, cur_y+PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
                if (cur_x<FrameLeft.cols-PATCHSIZE && cur_y<FrameLeft.rows-PATCHSIZE)     CheckConnection(cur_x+PATCHSIZE, cur_y+PATCHSIZE, cur_x, cur_y, idx, Q, checkFlag,inQueue,motionThreshold);
            }else{
                if (DEBUG) cout << "too far \n";

            }
        }else{
            lastStereoData[ZEDCACHESIZE-idx-1].MotionMask(cv::Rect(cv::Point2f(cur_x, cur_y), cv::Point2f(tgt_x, tgt_y)))   = 255;
        }
    }else{
        if (DEBUG) cout << "..."<< lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(tgt_y, tgt_x) << " ... NOT in mask\n";
        if ((int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(cur_y, cur_x) == 255){
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


void pauseStopResume(){
    if (DEBUG) cout << "pausing....\n";
    char new_key;

    while(new_key != 'r'){
        new_key = cv::waitKey(20);
        if (new_key == 'n') {
            PAUSE_FLAG = true;
            if (DEBUG) cout << "resuming....\n";
            return;
        }
    }
    if (DEBUG) cout << "resuming....\n";
    PAUSE_FLAG = false;
}
void processKey(char key){
    if (key == 'p' || PAUSE_FLAG) pauseStopResume();
//    if (key == 's') saveFrame();
}



void AugmentedVR::PC_segmentation(int idx){
    // TODO: get a good segmentation result, 2D or 3D;
    if (DEBUG>1) imshow("mask before segmentation", lastStereoData[ZEDCACHESIZE-idx-1].DynamicFrame);

//    Remove_HighLow(idx);


    cv::RNG rng(54321);
    cv::Mat checkFlag(FrameLeft.cols, FrameLeft.rows, CV_8U, cv::Scalar::all(0));
    cv::Mat inQueue(FrameLeft.cols, FrameLeft.rows, CV_8U, cv::Scalar::all(0));
    int x, y;
//    cout << lastStereoData[ZEDCACHESIZE-idx-1].MotionMask;
    float motionThreshold = 0.7*(idx+1);
    for (int i=0;i<3;i++){ //TODO: change to scan the matrix instead of dropping random pins
        while (true){
            x = rng.uniform(0, FrameLeft.cols+1);
            y = rng.uniform(0, FrameLeft.rows+1);

            int res = lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(y,x);

            cout << "PC_seg:: trying: " << x << "," << y << "..." << res << endl;
            if ((int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(y,x) == 255) break;
//            cv::Mat PCcopy(2,2,CV_8U, 255);
////            cout << (int)PCcopy.at<uchar>(0,0)<< endl;
//            PCcopy.at<uchar>(0,1) = 0;
//            cout << PCcopy;
//            cout << (int)PCcopy.at<uchar>(0,0)<< endl;

//            if (PCDisplacement.at<float>(x,y) < motionThreshold ) break;
        }
        cv::Point2f pt(x,y);
        std::queue<cv::Point2f> Q;
        Q.push(pt);
        cv::Point2f tmp;

        cv::namedWindow("current mask", cv::WINDOW_AUTOSIZE);

        while(!Q.empty()){
            tmp = Q.front();

            cout << "PC_seg:: expanding: " << tmp.x << "," << tmp.y << ", size: "<< FrameLeft.cols << "," << FrameLeft.rows << endl;

            if ((int)checkFlag.at<uchar>(tmp.y, tmp.x)==255){
                if (DEBUG) cout << "       ... already checked\n";
                inQueue.at<uchar>(tmp.y, tmp.x) = 0;
                Q.pop();
                continue;
            }

            checkFlag.at<uchar>(tmp.y, tmp.x) = 255;
            if (DEBUG){
                if ((int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(tmp.y, tmp.x) ==255){
                    cout << (int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(tmp.y, tmp.x) << "       ... in mask\n";
                }else{
                    cout << (int)lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.at<uchar>(tmp.y, tmp.x) << "       ... out of mask\n";
                }
            }
            //check each neighbor
            if (tmp.x>PATCHSIZE && tmp.y>PATCHSIZE)                    CheckConnection(tmp.x-PATCHSIZE, tmp.y-PATCHSIZE, tmp.x, tmp.y, idx, Q, checkFlag,inQueue, motionThreshold);
            if (tmp.x<FrameLeft.cols-PATCHSIZE && tmp.y>PATCHSIZE)     CheckConnection(tmp.x+PATCHSIZE, tmp.y-PATCHSIZE, tmp.x, tmp.y, idx, Q, checkFlag,inQueue,motionThreshold);
            if (tmp.x>PATCHSIZE && tmp.y<FrameLeft.rows-PATCHSIZE)                    CheckConnection(tmp.x-PATCHSIZE, tmp.y+PATCHSIZE, tmp.x, tmp.y, idx, Q, checkFlag,inQueue,motionThreshold);
            if (tmp.x<FrameLeft.cols-PATCHSIZE && tmp.y<FrameLeft.rows-PATCHSIZE)     CheckConnection(tmp.x+PATCHSIZE, tmp.y+PATCHSIZE, tmp.x, tmp.y, idx, Q, checkFlag,inQueue,motionThreshold);
            // mark checked

//            checkFlag(Rect(Point2f(tmp.y, tmp.x), )) = 255;
            inQueue.at<uchar>(tmp.y, tmp.x) = 0;
            Q.pop();

            cv::Mat mask;
            lastStereoData[ZEDCACHESIZE-idx-1].MotionMask.copyTo(mask);
            circle(mask, tmp, 2, 128, cv::FILLED, cv::LINE_AA);
            imshow("current mask", mask);
            char key = cv::waitKey(100);
            processKey(key);

        }


    }

    lastStereoData[ZEDCACHESIZE-idx-1].FrameLeft.copyTo(lastStereoData[ZEDCACHESIZE-idx-1].DynamicFrame, lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
    if (DEBUG>1) imshow("mask after segmentation", lastStereoData[ZEDCACHESIZE-idx-1].DynamicFrame);



}


//void AugmentedVR::PC_segmentation(){
//
//    Scalar colorTab[] =
//            {
//                    Scalar(0, 0, 255),
//                    Scalar(0,255,0),
//                    Scalar(255,100,100),
//                    Scalar(255,0,255),
//                    Scalar(0,255,255)
//            };
//
//    int clusterCount = 5;
//    cv::Mat labels, centers;
//    kmeans(pointcloud, clusterCount, labels,
//           TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
//           3, KMEANS_PP_CENTERS, centers);
//
//    cv::Mat img;
//    FrameLeft.copyTo(img);
//
//    for( int i = 0; i < img.rows; i++ ){
//        for (int j=0;j<img.cols; j++) {
//            int clusterIdx = labels.at<int>(i,j);
//            Point ipt(i,j);
//            circle(img, ipt, 2, colorTab[clusterIdx], FILLED, LINE_AA);
//        }
//    }
//    imshow("clustering result", img);
//
//}
void AugmentedVR::filterObjectMotion(int idx){
////    cv::Mat img_obj,img_scene;
//    cv::Mat curDynamicFrame, lastDynamicFrame;
//    cv::Mat rawcurDynamicFrame, rawlastDynamicFrame;
//    FrameLeft.copyTo(rawcurDynamicFrame,lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
//    lastStereoData[ZEDCACHESIZE-idx-1].DynamicFrame.copyTo(rawlastDynamicFrame,lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
//    cv::cvtColor(rawlastDynamicFrame, lastDynamicFrame, CV_BGR2GRAY);
//    cv::cvtColor(rawcurDynamicFrame, curDynamicFrame, CV_BGR2GRAY);
////    cv::cvtColor(lastStereoData[ZEDCACHESIZE-idx-1].FrameLeft, img_obj, CV_BGR2GRAY);
////    cv::cvtColor(FrameLeft, img_scene, CV_BGR2GRAY);
//
////    std::vector<Point2f> obj;
////    std::vector<Point2f> scene;
////    std::vector<Point2f> obj_corners(4);
////    std::vector<Point2f> scene_corners(4);
//
//    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//    Size subPixWinSize(10,10), winSize(31,31);
//
////    Mat curDynamicFrame, lastDynamicFrame, image, frame;
//    vector<Point2f> points[2], points_trans;
//
//    const int MAX_COUNT = 500;
//    cv::goodFeaturesToTrack(lastDynamicFrame, points[0], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
//    cornerSubPix(lastDynamicFrame, points[0], subPixWinSize, Size(-1,-1), termcrit);
//
//    vector<uchar> status;
//    vector<float> err;
//
//    calcOpticalFlowPyrLK(lastDynamicFrame, curDynamicFrame, points[0], points[1], status, err, winSize,
//                         3, termcrit, 0, 0.001);
////    size_t i, k;
////    for( i = k = 0; i < points[1].size(); i++ )
////    {
////        if( !status[i] )
////            continue;
////
//////        points[1][k++] = points[1][i];
//////        points[0][k++] = points[0][i];
////        circle( rawcurDynamicFrame, points[1][i], 3, Scalar(0,255,0), -1, 8);
////        circle( rawlastDynamicFrame, points[0][i], 3, Scalar(0,255,0), -1, 8);
////    }
//////    points[1].resize(k);
//////    points[0].resize(k);
////
////    imshow( "Cur.jpg", rawcurDynamicFrame );
////    imshow( "Prev.jpg", rawlastDynamicFrame);

    vector<cv::Point2f> points_trans;

    cv::Mat img;

    LastFrame.FrameLeft.copyTo(img);

//    img = mSLAM->DrawSlamFrame();


    if (!lastStereoData[ZEDCACHESIZE-1-idx].sceneTransformMat.empty()&& !lastStereoData[ZEDCACHESIZE - 1 - idx].MotionMask.empty()){

        perspectiveTransform( lastStereoData[ZEDCACHESIZE-1-idx].keypoints, points_trans, lastStereoData[ZEDCACHESIZE-1-idx].sceneTransformMat);

        cv::Mat total_motionVec(1,1,CV_32FC3,cv::Scalar(0.,0.,0.));
        int count = 0;


        for( size_t i = 0; i < points_trans.size(); i++ ) {
            cv::Rect rect(0, 0, img.cols, img.rows);
            if (rect.contains(LastFrame.keypoints[i])  && rect.contains(lastStereoData[ZEDCACHESIZE - 1 - idx].keypoints[i])) {
                // check if the point is in range after transformation
                if (LastFrame.tracked_status[i] && norm(points_trans[i] - LastFrame.keypoints[i]) > 5) {

                    if (DEBUG) {

                        circle(img, lastStereoData[ZEDCACHESIZE - 1 - idx].keypoints[i], 3, cv::Scalar(255, 255, 0), -1, 8);
                        circle(img, points_trans[i], 3, cv::Scalar(255, 0, 0), -1, 8);
                        circle(img, LastFrame.keypoints[i], 3, cv::Scalar(0, 255, 0), -1, 8);
                        line(img, points_trans[i], LastFrame.keypoints[i], cv::Scalar(0, 0, 255));
                        line(img, lastStereoData[ZEDCACHESIZE - 1 - idx].keypoints[i], points_trans[i],
                             cv::Scalar(0, 255, 255));
                    }

                    if (lastStereoData[ZEDCACHESIZE - 1 - idx].MotionMask.at<uchar>(lastStereoData[ZEDCACHESIZE - 1 - idx].keypoints[i]) == 255 &&
                        lastStereoData[ZEDCACHESIZE - 1 - idx].MotionMask.at<uchar>(LastFrame.keypoints[i]) == 255) {


                        //                    cout << lastStereoData[ZEDCACHESIZE-1-idx].PC_noColor(Rect(lastStereoData[ZEDCACHESIZE-1-idx].keypoints[i].y,lastStereoData[ZEDCACHESIZE-1-idx].keypoints[i].y,1,1));
                        //                    cout << lastStereoData[ZEDCACHESIZE-1-idx].PC_noColor.at<Vec3f>(lastStereoData[ZEDCACHESIZE-1-idx].keypoints[i])  << endl;
                        //                    cout << PC_noColor(Rect(keypoints[i].y,keypoints[i].y,1,1));
                        //                    cout << PC_noColor.at<Vec3f>(keypoints[i]) << endl;


                        cv::Mat motionVec = lastStereoData[ZEDCACHESIZE - 1 - idx].PC_noColor(
                                cv::Rect(lastStereoData[ZEDCACHESIZE - 1 - idx].keypoints[i].x,
                                     lastStereoData[ZEDCACHESIZE - 1 - idx].keypoints[i].y, 1, 1)) -
                                LastFrame.PC_noColor(cv::Rect(LastFrame.keypoints[i].x, LastFrame.keypoints[i].y, 1, 1));
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
            img.copyTo(masked_img,lastStereoData[ZEDCACHESIZE-1-idx].MotionMask);
            imshow("masked KLT matches", masked_img);
        }

        total_motionVec /= count;
        if (DEBUG)
            cout << "Total Motion Vec: " << total_motionVec << " >> " << norm(total_motionVec)<< endl;
        total_motionVec.copyTo(ObjectMotionVec);
        // TODO: find a safe way to do it
        ObjectMotionVec.copyTo(lastStereoData[ZEDCACHESIZE-1].ObjectMotionVec);

        // low pass filtering (sliding window average)
        cv::Mat lp_total(1,1,CV_32FC3,cv::Scalar(0.,0.,0.));
        for (int i=0;i<ZEDCACHESIZE-1;i++){
            if (!lastStereoData[ZEDCACHESIZE-1-i].ObjectMotionVec.empty()){
                lp_total+= lastStereoData[ZEDCACHESIZE-1-i].ObjectMotionVec;
            }
        }
        lp_total /= ZEDCACHESIZE-1;
        lp_total.copyTo(Log_LowPassMotionVec);
        Log_LowPassMotionVec.copyTo(lastStereoData[ZEDCACHESIZE-1].LowPass_ObjectMotionVec);
        if (DEBUG) cout << "Low Pass Total Motion Vec: " << Log_LowPassMotionVec<< " >> " << norm(Log_LowPassMotionVec)<< endl;
    }
}


//void AugmentedVR::filterObjectMotion(int idx){
//
//
//
//    cv::Mat img_obj,img_scene;
//
//    cv::Mat curDynamicFrame, lastDynamicFrame;
//    FrameLeft.copyTo(curDynamicFrame,lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
//    lastStereoData[ZEDCACHESIZE-idx-1].DynamicFrame.copyTo(lastDynamicFrame,lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
//    cv::cvtColor(lastDynamicFrame, img_obj, CV_BGR2GRAY);
//    cv::cvtColor(curDynamicFrame, img_scene, CV_BGR2GRAY);
//
//
////    cv::cvtColor(lastStereoData[ZEDCACHESIZE-idx-1].FrameLeft, img_obj, CV_BGR2GRAY);
////    cv::cvtColor(FrameLeft, img_scene, CV_BGR2GRAY);
//
//    std::vector<Point2f> obj;
//    std::vector<Point2f> scene;
//    std::vector<Point2f> obj_corners(4);
//    std::vector<Point2f> scene_corners(4);
//
////    if( !img_object.data || !img_scene.data )
////  { std::cout<< " --(!) Error reading images " << std::endl; return NULL; }
//    //-- Step 1: Detect the keypoints and extract descriptors using SURF
//    int minHessian = 400;
////    Ptr<SURF> detector = SURF::create( minHessian );
//    Ptr<GFTTDetector> detector = GFTTDetector::create( );
////    Ptr<ORB> detector = ORB::create( minHessian );
//
//    // SurfFeatureDetector detector( minHessian );
//    std::vector<KeyPoint> keypoints_object, keypoints_scene;
//    cv::Mat descriptors_object, descriptors_scene;
//    detector->detectAndCompute( img_obj, cv::Mat(), keypoints_object, descriptors_object );
//    detector->detectAndCompute( img_scene, cv::Mat(), keypoints_scene, descriptors_scene );
//    //-- Step 2: Matching descriptor vectors using FLANN matcher
//    FlannBasedMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_object, descriptors_scene, matches );
//    double max_dist = 0; double min_dist = 100;
//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < descriptors_object.rows; i++ )
//    { double dist = matches[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }
////    if(DEBUG){
//        printf("-- Max dist : %f \n", max_dist );
//        printf("-- Min dist : %f \n", min_dist );
////    }
//    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< DMatch > good_matches;
////    double min_dist = 100;
//    for( int i = 0; i < descriptors_object.rows; i++ )
//    { if( matches[i].distance < 5* min_dist )
//        { good_matches.push_back( matches[i]); }
//    }
//    cv::Mat img_matches;
//    drawMatches( img_obj, keypoints_object, img_scene, keypoints_scene,
//                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                 std::vector<char>(), DrawMatchesFlags::DEFAULT );
//    for( size_t i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//    }
////    cv::resize(img_matches,img_matches,DisplaySize);
//
////    img_matches.copyTo(img_matches, lastStereoData[ZEDCACHESIZE-idx-1].MotionMask);
//    imshow( "Moving Object detection", img_matches );
//
//}


//TODO: move all these to PCManipulator
void AugmentedVR::shiftPC(cv::Mat pc1,cv::Scalar vec){
    cv::Mat shift(pc1.rows, pc1.cols, CV_32FC4, vec);
    pc1+=shift;
}

void AugmentedVR::saveCurFrame() {
    char tmp_str[50];
    sprintf(tmp_str, "./cam%d_frame%d_ts%ld_left.png", CamId, frameSeq, frameTS);
    imwrite(tmp_str,FrameLeft);
}

void AugmentedVR::setMSLAM(ORB_SLAM2::System *mSLAM) {
    AugmentedVR::mSLAM = mSLAM;
}

ORB_SLAM2::System *AugmentedVR::getMSLAM() const {
    return mSLAM;
}

int AugmentedVR::getCamId() const {
    return CamId;
}

long AugmentedVR::getStartTS() const {
    return startTS;
}

long AugmentedVR::getFrameTS() const {
    return frameTS;
}

long AugmentedVR::getZEDTS() const {
    return ZEDTS;
}

int AugmentedVR::getFrameSeq() const {
    return frameSeq;
}
