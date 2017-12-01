//
// Created by hang on 3/2/17.
//

#include <sys/time.h>
#include <include/UtilsCV.hpp>
#include <include/UtilsPC.hpp>
#include <include/OccupancyGrid.hpp>
#include "VCluster.hpp"
using namespace sl;

//void VCluster::initDisplay(){
//    if (VISUAL) mDisplayer = new Displayer(VNode);
//}

VCluster::VCluster(const string mapFile, string VPath="") {
    frameSeqRx = 220;
    timeRx=0;

    InitParameters init_parameters;
//    init_parameters.camera_resolution = RESOLUTION::RESOLUTION_HD720;
    init_parameters.coordinate_units = UNIT_METER; // set meter as the OpenGL world will be in meters
    init_parameters.sdk_verbose = 1;
    /// ZED default is the same with ORBSLAM
//    init_parameters.coordinate_system = COORDINATE_SYSTEM::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
//    init_parameters.coordinate_system = COORDINATE_SYSTEM::COORDINATE_SYSTEM_LEFT_HANDED_Y_UP; // Unity's coordinate system is right_handed
    if(VPath != "live"){
        /// footage mode
        cout << "Footage Mode" << endl;
        init_parameters.svo_input_filename = VPath.c_str();
        init_parameters.depth_mode = DEPTH_MODE::DEPTH_MODE_QUALITY; //need quite a powerful graphic card in QUALITY[
    }else{
        /// live mode
        live = true;
        cout << "Live Mode" << endl;
        init_parameters.depth_mode = DEPTH_MODE::DEPTH_MODE_PERFORMANCE;
        init_parameters.camera_resolution = RESOLUTION::RESOLUTION_VGA;
        init_parameters.camera_fps = 15; /// lowest possible
        SHOW_CAMMOTION = false;
    }

    RuntimeParameters runtime_parameters;
//    runtime_parameters.sensing_mode = SENSING_MODE_FILL;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    int ZEDConfidence = 85;
    VNode = new AugmentedVR* [NUM_CAMERAS];

    int i=0;
    // Initialization
    VNode[i] = new AugmentedVR(CamId, init_parameters, runtime_parameters, ZEDConfidence);
    VNode[i]->initZEDCam(startFrameId);

    if (ReuseMap){
        cout << "Reusing Map\n";
        VNode[i]->initSLAMStereo(VocFile, CalibrationFile,true, mapFile);
    }
    else{
        VNode[i]->initSLAMStereo(VocFile, CalibrationFile,false);
    }

    if (VISUAL) mDisplayer = new Displayer(VNode);

    if (VehicleControl) {
        mPathPlanner = new PathPlanner(0.5,0.5,-7,2,-5,40);
    }



    if (TX) mSender = new ObjSender(VNode[0], commPath);
    if (RX) mReceiver = new ObjReceiver(VNode[0], RxCamId, commPath);

    mCodec = new pcCodec(VNode[0]->width,VNode[0]->height);
}

void VCluster::run(){
#ifdef PIPELINE
    thread CPU_download;
#endif

    char key = ' ';
    int count = 0;
#ifdef EVAL
    timeval tTotalStart, tFetchStart, tCacheStart, tSlamStart, tPCMotionStart, tPCMotionFilterStart, tObjectFilterStart, tTXStart, tRXStart, tPCmergeStart,tDeadReckonStart;
    timeval tTotalEnd, tFetchEnd, tCacheEnd, tSlamEnd, tPCMotionEnd, tPCMotionFilterEnd, tObjectFilterEnd, tTXEnd, tRXEnd, tPCmergeEnd, tDeadReckonEnd;
    timeval tInit;
    gettimeofday(&tInit, NULL);
    double prepFrameTime=0;
    double slamTime=0;
    double totalTime=0;
#endif
    ///loop until 'q' is pressed
    ////////////////////////////////////////////////////////////// main loop/////////////////////////////////////////////////
#ifdef SIMPLEEVAL
    timeval FrameStartT, LastFrameStartT;
    gettimeofday(&FrameStartT, NULL);
    gettimeofday(&LastFrameStartT, NULL);
    cout << "Total: " <<timeDifference_msec(LastFrameStartT, FrameStartT) << "ms"<< endl;
#endif
    while (key != 'q' && !quit && (VNode[0]->TotalFrameSeq < lengthInFrame || live )) {
//        key = waitKey(20);
        FRAME_ID = VNode[0]->TotalFrameSeq;
        count++;
#ifdef SIMPLEEVAL
        /// calc frame rate
        gettimeofday(&FrameStartT, NULL);
        double frameTime = timeDifference_msec(LastFrameStartT, FrameStartT);
        double frameRate = 1/frameTime *1000;
        cout << "Total: " << frameTime<< "ms, "<< frameRate << "fps" << endl;
        LastFrameStartT = FrameStartT;
#endif

        if (VISUAL) key = mDisplayer->showCurFrame();

#ifdef EVAL
        gettimeofday(&tFetchStart, NULL);
        gettimeofday(&tTotalStart, NULL);
#endif
        /// break when footage runs out, not break for live mode
        if (!live && !VNode[0]->grabNextZEDFrameOffline() ) break;
        /// sync all thread, must run before any thread fork out
        VNode[0]->SinkFrames();

        if (!RX) cout << endl << "Current FrameID, " << VNode[0]->TotalFrameSeq-2<< ", "<< VNode[0]->getCurrentAVRFrame_TimeStamp_FrameTS()<<  endl;

#ifdef PIPELINE
        if (CPU_download.joinable()) CPU_download.join();
        CPU_download = thread(&VCluster::PreProcess, this);
#else
        if (!PreProcess()) break;
#endif
#ifdef EVAL
        gettimeofday(&tFetchEnd, NULL);
        cout << "TimeStamp: " << timeDifference_msec(tInit,tFetchEnd) << "ms: ";
        cout << "run >>>> Prepare Frame: " << timeDifference_msec(tFetchStart,tFetchEnd) << "ms" << endl;
        gettimeofday(&tSlamStart, NULL);
        prepFrameTime += timeDifference_msec(tFetchStart,tFetchEnd);
#endif
        VNode[0]->trackCam();

#ifdef EVAL
        gettimeofday(&tSlamEnd, NULL);
        cout << "TimeStamp: " << timeDifference_msec(tInit,tSlamEnd) << "ms: ";
        cout << "run >>>> SLAM: " << timeDifference_msec(tSlamStart,tSlamEnd) << "ms" << endl;
        gettimeofday(&tCacheStart, NULL);
        slamTime += timeDifference_msec(tSlamStart,tSlamEnd);
#endif
        postProcess();
#ifdef EVAL
        gettimeofday(&tTotalEnd, NULL);
        cout << "TimeStamp: " << timeDifference_msec(tInit,tTotalEnd)<< "ms: ";
        cout << "Total: " <<timeDifference_msec(tTotalStart,tTotalEnd)<< "ms"<< endl;
        totalTime += timeDifference_msec(tTotalStart,tTotalEnd);
        cout << "Avg prep: " << prepFrameTime / count << ", slam: " << slamTime / count << ", total: " << totalTime / count << endl;
#endif
    }

    if (!quit){

        VNode[0]->mSLAM->SaveMap("Slam_latest_Map.bin");
    }
}


VCluster::~VCluster(){
    for (int i=0;i<NUM_CAMERAS;i++){
        delete VNode[i];
    }
    if (VISUAL) delete mDisplayer;
    if (TX)    delete mSender;
    if (RX)    delete mReceiver;
}

void VCluster::exit(){
    if (VISUAL)  mDisplayer->exit();
    for (int i=0;i<NUM_CAMERAS;i++){
        VNode[i]->exit();
    }
}

bool VCluster::PreProcess(){

#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
    cout << "TimeStamp: " << double(start.tv_sec-tInit.tv_sec)*1000 + double(start.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "PreProcess starts" << endl;
#endif


    VNode[0]->FeedSlamTheSlamFrame();


#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "PreProcess ends: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
#endif
    return true;
}

void VCluster::compressDynamic(){
//    cout << "compressing \n ";
    cv::Mat tmp;
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    currFrame.DynamicPC.copyTo(tmp);
    mCodec->encode(tmp);
}

void VCluster::RoadDetection(){

    /// merge PC
    cv::Mat totalPC;
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    if (RX){
        hconcat(currFrame.pointcloud, VNode[0]->transRxPC,totalPC);
    }else{
        currFrame.pointcloud.copyTo(totalPC);
    }

    cv::Mat tmp;
//    removePointCloud_HighLow(totalPC,tmp);
    totalPC.copyTo(tmp);
    sl::Mat slpc;
//    mCodec->planeSegmentation(tmp, slpc);
//    mCodec->planeSegmentation_ManualPlaneModel(tmp, slpc, -0.1, -0.924138, 0.17, 1.8);/// FOR PSA ROOF FOOTAGE
//    OccupancyGrid mGrid(1,1,-5.5,5.5,0,20);
//    mGrid.ConvertPCAndSetOccupancyGrid_ManualPlaneModel(tmp, slpc, -0.1, -0.924138, 0.17, 1.8);/// FOR PSA ROOF FOOTAGE
    int HorizonZMin = 6, HorizonZMax = 18;
    if (RX){
        HorizonZMin = 6;
        HorizonZMax = 25;
    }
    mPathPlanner->PlanPath_AStar_ManualRoadModel(currFrame.frameSeq, tmp,slpc,-1,4,-2,30,-0.1, -0.924138, 0.2, 1.1,HorizonZMax,HorizonZMin);/// FOR PSA ROOF FOOTAGE, proves to be working on BMW roof mount, pretty robust and consistent

    mDisplayer->pushPC_slMat_CPU(slpc);
}

//void VCluster::segmentation(){
////    cout << "compressing \n ";
//    cv::Mat tmp;
//    VNode[0]->pointcloud.copyTo(tmp);
////    mCodec->euclideanSegmentation(tmp);
//}

void VCluster::postProcess(){

//    VNode[0]->trackCam();

#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
    cout << "TimeStamp: " << double(start.tv_sec-tInit.tv_sec)*1000 + double(start.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "postProcess starts" << endl;
#endif

    VNode[0]->calcOpticalFlow();
    VNode[0]->analyze();
//    compressDynamic();

    if (OfflineTXRX){
        TXRX_viaDisk();
    }else{
        TXRX();
    }

    if (VISUAL){
        visualize();
    }
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "postProcess ends: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
#endif
    if (DECOUPLE2IMG) VNode[0]->mIo->writeCurrentStereoFrame();
    if (DEBUG) VNode[0]->mIo->logCurrentFrame();
}



void VCluster::TXRX(){


    if (TX && SEND) {
        /// sending objects
        if (!Parallel_TXRX){
            mSender->StreamPointCloud();
        }else{
            mSender->StreamPointCloud_Async();
        }
    }


    if (RX){
        /// receiving objects
        if (!Parallel_TXRX){
            mReceiver->ReceivePointCloudStream();
        }else{
            /// parallel background RX
            RxFrame* rx = VNode[0]->RxBuffer.getCurrentRxFrame();
            cout << "Current FrameID, " << VNode[0]->TotalFrameSeq-2
                 << ", "<< VNode[0]->getCurrentAVRFrame_TimeStamp_FrameTS() / 1000
                 <<","<< VNode[0]->getCurrentAVRFrame_AbsoluteTimeStamp() / 1000000 ;
            cout << ", Received Frame, " << rx->RxSeq
                 << ", " << rx->RxTimeStamp /1000
                 << ", " << rx->RxTimeStamp_ZEDTS /1000000
                 << endl;
        }
    }
}


void VCluster::TXRX_viaDisk(){

#ifdef EVAL

    timeval tTotalStart, tFetchStart, tCacheStart, tSlamStart, tPCMotionStart, tPCMotionFilterStart, tObjectFilterStart, tTXStart, tRXStart, tPCmergeStart,tDeadReckonStart;
    timeval tTotalEnd, tFetchEnd, tCacheEnd, tSlamEnd, tPCMotionEnd, tPCMotionFilterEnd, tObjectFilterEnd, tTXEnd, tRXEnd, tPCmergeEnd, tDeadReckonEnd;
#endif
    if (DEBUG) VNode[0]->mIo->logCurrentFrame();

    // sending objects
    if (TX && SEND) {
#ifdef EVAL
        gettimeofday(&tTXStart, NULL);
#endif
        mSender->writeFrameInSeparateFile();
#ifdef EVAL
        gettimeofday(&tTXEnd, NULL);
        cout << "TimeStamp: " << double(tTXEnd.tv_sec-tInit.tv_sec)*1000 + double(tTXEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "TXRX >>>>> TX: " <<double(tTXEnd.tv_sec-tTXStart.tv_sec)*1000 + double(tTXEnd.tv_usec-tTXStart.tv_usec) / 1000<< "ms"<< endl;
#endif
    }

    /// ensure atomic reception
    if (RX){
        // receiving objects
        // searcing for synced frame
        frameSeqRx ++;
        mReceiver->ReadFromDisk(frameSeqRx);


    }
}


void VCluster::visualize(){
    // need to show PC from Last Frame, cause buffer are freed for pre-fetching
    // Point Cloud Stiching
    if (VISUAL && PCVISUAL ) {
//        mDisplayer->showCurFrame();
        AVRFrame currFrame;
        VNode[0]->getCurrentAVRFrame(currFrame);
        if (TX) {
//                mDisplayer->showPC(VNode[0]->lastStereoData[ZEDCACHESIZE-1].DynamicPC);

            mDisplayer->showPC(currFrame.pointcloud);
//            saveOpenGL(1000, 1000);
        }
        else if (RX) {
            RxFrame* rx = VNode[0]->RxBuffer.getCurrentRxFrame();
            if (VNode[0]->trackGood() && !(rx->RxTCW.empty())){


                /// calculating rela position
                cv::Mat Trc;
                VNode[0]->calcRelaCamPos(rx->RxTCW, Trc);

                /// PC manipulation
                VNode[0]->transformRxPCtoMyFrameCoord(Trc, rx->RxPC, VNode[0]->transRxPC);
                if (DYNAMICS){
                    VNode[0]->transformRxPCtoMyFrameCoord(Trc, rx->RxDynamicPC, VNode[0]->transRxDynamicPC);
                }

                /// received frame feature matching with curr frame, eval only
//                VNode[0]->TransPCvsPC();
                VNode[0]->TransPCvsPC(rx->RxTCW, rx->RxFrameLeft, rx->RxMotionVec, rx->RxTimeStamp);
            }
            /// Dead-Reckoning
//            else{
//
//                if ( !VNode[0]->RxMotionVec.empty()){
//#ifdef EVAL
//                    gettimeofday(&tDeadReckonStart, NULL);
//#endif
//                    VNode[0]->dead_reckoning_onRxDynamicPC();
//#ifdef EVAL
//                    gettimeofday(&tDeadReckonEnd, NULL);
//                cout << "TimeStamp: " << double(tDeadReckonEnd.tv_sec-tInit.tv_sec)*1000 + double(tDeadReckonEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
//                cout << "TXRX >>>>> Dead reckon: " <<double(tDeadReckonEnd.tv_sec-tDeadReckonStart.tv_sec)*1000 + double(tDeadReckonEnd.tv_usec-tDeadReckonStart.tv_usec) / 1000<< "ms"<< endl;
//#endif
//                }
//
//            }

            /// visualize
            if (!(VNode[0]->transRxPC.empty()) ) { //&&  !(VNode[0]->transRxDynamicPC.empty())

                if (VehicleControl) {
                    RoadDetection();
                }else{
                    mDisplayer->showMergedPC(VNode[0]->transRxPC);
                }
            }


            if(COOP){
                cv::Mat nonOverlapingPC;
                VNode[0]->removeOverlap(VNode[0]->transRxDynamicPC).copyTo(nonOverlapingPC);
//                    mDisplayer->showSplitScreen(VNode[0]->transRxPC,nonOverlapingPC);
                mDisplayer->showPC(nonOverlapingPC);
//                    mDisplayer->showPC(VNode[0]->transRxPC);
            }
            //        if (VISUAL && SHOW_PC) mDisplayer->showImgWithPC(RxFrame,  &(VNode[0]->transRxDynamicPC), "rx trans PC");
//            }
        }
        else{
            if (VehicleControl){
                RoadDetection();
            }else{
                mDisplayer->showPC(currFrame.pointcloud);
            }
        }


    }
}
//void VCluster::visualize(){
//    // need to show PC from Last Frame, cause buffer are freed for pre-fetching
//    // Point Cloud Stiching
//    if (VISUAL && SHOW_PC ) {
////        mDisplayer->showCurFrame();
//        AVRFrame currFrame;
//        VNode[0]->getCurrentAVRFrame(currFrame);
//        if (TX) {
////                mDisplayer->showPC(VNode[0]->lastStereoData[ZEDCACHESIZE-1].DynamicPC);
//
//            mDisplayer->showPC(currFrame.pointcloud);
////            saveOpenGL(1000, 1000);
//        }
//        else if (RX) {
//            RxFrame* rx = VNode[0]->RxBuffer.getCurrentRxFrame();
//            if (VNode[0]->trackGood() && !(VNode[0]->RxTCW.empty())){
//
//
//                /// calculating rela position
//                cv::Mat Trc;
//                VNode[0]->calcRelaCamPos(VNode[0]->RxTCW, Trc);
//
//                /// PC manipulation
//                VNode[0]->transformRxPCtoMyFrameCoord(Trc, VNode[0]->RxPC, VNode[0]->transRxPC);
//                if (DYNAMICS){
//                    VNode[0]->transformRxPCtoMyFrameCoord(Trc, VNode[0]->RxDynamicPC, VNode[0]->transRxDynamicPC);
//                }
//
//                /// received frame feature matching with curr frame, eval only
//                VNode[0]->TransPCvsPC();
//            }
//            /// Dead-Reckoning
////            else{
////
////                if ( !VNode[0]->RxMotionVec.empty()){
////#ifdef EVAL
////                    gettimeofday(&tDeadReckonStart, NULL);
////#endif
////                    VNode[0]->dead_reckoning_onRxDynamicPC();
////#ifdef EVAL
////                    gettimeofday(&tDeadReckonEnd, NULL);
////                cout << "TimeStamp: " << double(tDeadReckonEnd.tv_sec-tInit.tv_sec)*1000 + double(tDeadReckonEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
////                cout << "TXRX >>>>> Dead reckon: " <<double(tDeadReckonEnd.tv_sec-tDeadReckonStart.tv_sec)*1000 + double(tDeadReckonEnd.tv_usec-tDeadReckonStart.tv_usec) / 1000<< "ms"<< endl;
////#endif
////                }
////
////            }
//
//            /// visualize
//            if (!(VNode[0]->transRxPC.empty()) ) { //&&  !(VNode[0]->transRxDynamicPC.empty())
//
//                mDisplayer->showMergedPC(VNode[0]->transRxPC);
////                saveOpenGL(VNode[0]->width*2, VNode[0]->height);
////                cv::Mat totalPC, totalDynamicPC;
////                hconcat(VNode[0]->getCurrentAVRFrame().pointcloud, VNode[0]->RxPC, totalPC);
////                hconcat(VNode[0]->pointcloud, transRxDynamicPC, totalDynamicPC);
////                  hconcat(VNode[0]->initPC, VNode[0]->transRxDynamicPC, totalDynamicPC);
////                mDisplayer->showSplitScreen(VNode[0]->getCurrentAVRFrame().pointcloud,totalPC);
////                mDisplayer->showSplitScreen(VNode[0]->pointcloud,totalDynamicPC);
////            mDisplayer->showPC(VNode[0]->transRxDynamicPC);
////                mDisplayer->showPC(VNode[0]->transRxPC);
////                mDisplayer->showPC(totalDynamicPC);
//
////            mDisplayer->showPC(VNode[0]->transRxPC);
//            }
//
//            if(COOP){
//                cv::Mat nonOverlapingPC;
//                VNode[0]->removeOverlap(VNode[0]->transRxDynamicPC).copyTo(nonOverlapingPC);
////                    mDisplayer->showSplitScreen(VNode[0]->transRxPC,nonOverlapingPC);
//                mDisplayer->showPC(nonOverlapingPC);
////                    mDisplayer->showPC(VNode[0]->transRxPC);
//            }
//            //        if (VISUAL && SHOW_PC) mDisplayer->showImgWithPC(RxFrame,  &(VNode[0]->transRxDynamicPC), "rx trans PC");
////            }
//        }
//        else{
//            // debug
//
//            mDisplayer->showPC(currFrame.pointcloud);
//
////            mDisplayer->showPC(VNode[0]->pointcloud_sl_gpu);
//        }
//    }
//}