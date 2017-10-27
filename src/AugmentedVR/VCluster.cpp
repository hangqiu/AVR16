//
// Created by hang on 3/2/17.
//

#include <sys/time.h>
#include <include/UtilsCV.hpp>
#include "VCluster.hpp"
using namespace sl;

//void VCluster::initDisplay(){
//    if (VISUAL) mDisplayer = new Displayer(VNode);
//}

VCluster::VCluster(bool live, const string mapFile, int argc, char** argv, string VPath="") {
    frameSeqRx = 0;
    timeRx=0;

    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::RESOLUTION_HD720;
    init_parameters.depth_mode = DEPTH_MODE::DEPTH_MODE_QUALITY; //need quite a powerful graphic card in QUALITY
    init_parameters.coordinate_units = UNIT_METER; // set meter as the OpenGL world will be in meters
    init_parameters.sdk_verbose = 1;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    init_parameters.svo_input_filename = VPath.c_str();

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




    if (TX) mSender = new ObjSender(VNode[0], commPath);
    if (RX) mReceiver = new ObjReceiver(VNode[0], RxCamId, commPath);

    mCodec = new pcCodec(VNode[0]->width,VNode[0]->height);
}

void VCluster::run(){
#ifdef PIPELINE
    thread analyze;
#endif


//    std::thread* pauser = new thread(debugCin);
//    debugPauser();

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
    //loop until 'q' is pressed
    ////////////////////////////////////////////////////////////// main loop/////////////////////////////////////////////////
    while (key != 'q' && !quit && VNode[0]->TotalFrameSeq < lengthInFrame) {
//        key = waitKey(20);
        FRAME_ID = VNode[0]->TotalFrameSeq;
        count++;
        //Resize and imshow
        cout << endl << "FrameID: " << FRAME_ID << endl;

        if (VISUAL) key = mDisplayer->showCurFrame();

#ifdef EVAL
        gettimeofday(&tFetchStart, NULL);
        gettimeofday(&tTotalStart, NULL);
#endif
#ifdef SIMPLEEVAL
        timeval tTotalStart, tTotalEnd;
        gettimeofday(&tTotalStart, NULL);
#endif

        VNode[0]->SinkFrames();
#ifdef PIPELINE
        thread CPU_download(&VCluster::PreProcess, this);
#else
        if (!PreProcess()) break;
#endif
#ifdef EVAL
        gettimeofday(&tFetchEnd, NULL);
        cout << "TimeStamp: " << double(tFetchEnd.tv_sec-tInit.tv_sec)*1000 + double(tFetchEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "run >>>> Prepare Frame: " << double(tFetchEnd.tv_sec-tFetchStart.tv_sec)*1000 + double(tFetchEnd.tv_usec-tFetchStart.tv_usec) / 1000 << "ms" << endl;
        gettimeofday(&tSlamStart, NULL);

        prepFrameTime += double(tFetchEnd.tv_sec-tFetchStart.tv_sec)*1000 + double(tFetchEnd.tv_usec-tFetchStart.tv_usec) / 1000;
#endif
        VNode[0]->trackCam();
#ifdef EVAL
        gettimeofday(&tSlamEnd, NULL);
        cout << "TimeStamp: " << double(tSlamEnd.tv_sec-tInit.tv_sec)*1000 + double(tSlamEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "run >>>> SLAM: " << double(tSlamEnd.tv_sec-tSlamStart.tv_sec)*1000 + double(tSlamEnd.tv_usec-tSlamStart.tv_usec) / 1000<< "ms" << endl;
        gettimeofday(&tCacheStart, NULL);
        slamTime += double(tSlamEnd.tv_sec-tSlamStart.tv_sec)*1000 + double(tSlamEnd.tv_usec-tSlamStart.tv_usec) / 1000;
#endif
#ifdef PIPELINE
        CPU_download.join();

        if (analyze.joinable()){analyze.join();}
#endif

        /// have to STAGE CURRENT FRAME first! to prevent data race with cpu download thread
#ifdef EVAL
        gettimeofday(&tCacheStart, NULL);
#endif



#ifdef EVAL
        gettimeofday(&tCacheEnd, NULL);
        cout << "TimeStamp: " << double(tCacheEnd.tv_sec-tInit.tv_sec)*1000 + double(tCacheEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "run >>>> Cache to 3rd stage, Postprocess: " << double(tCacheEnd.tv_sec-tCacheStart.tv_sec)*1000 + double(tCacheEnd.tv_usec-tCacheStart.tv_usec) / 1000<< "ms" << endl;
#endif

#ifdef PIPELINE
        analyze = thread(&VCluster::postProcess,this);
#else
        postProcess();
#endif


#ifdef EVAL
        gettimeofday(&tTotalEnd, NULL);
        cout << "TimeStamp: " << double(tTotalEnd.tv_sec-tInit.tv_sec)*1000 + double(tTotalEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "Total: " <<double(tTotalEnd.tv_sec-tTotalStart.tv_sec)*1000 + double(tTotalEnd.tv_usec-tTotalStart.tv_usec) / 1000<< "ms"<< endl;
        totalTime += double(tTotalEnd.tv_sec-tTotalStart.tv_sec)*1000 + double(tTotalEnd.tv_usec-tTotalStart.tv_usec) / 1000;
        cout << "Avg prep: " << prepFrameTime / count << ", slam: " << slamTime / count << ", total: " << totalTime / count << endl;
#endif
#ifdef SIMPLEEVAL
        gettimeofday(&tTotalEnd, NULL);
        //cout << "TimeStamp: " << double(tTotalEnd.tv_sec-tInit.tv_sec)*1000 + double(tTotalEnd.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
        cout << "Total: " <<double(tTotalEnd.tv_sec-tTotalStart.tv_sec)*1000 + double(tTotalEnd.tv_usec-tTotalStart.tv_usec) / 1000<< "ms"<< endl;
        totalTime += double(tTotalEnd.tv_sec-tTotalStart.tv_sec)*1000 + double(tTotalEnd.tv_usec-tTotalStart.tv_usec) / 1000;
        cout << "Avg total: " << totalTime / count << endl;
#endif
    }

    if (!quit){

        VNode[0]->mSLAM->SaveMap("Slam_latest_Map.bin");
    }
#ifdef PIPELINE
    analyze.join();
#endif
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
    VNode[0]->calcOpticalFlow();
    if (!(VNode[0]->PrepareNextFrame())) return false;
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
    currFrame.pointcloud.copyTo(tmp);
    mCodec->encode(tmp);
}


//void VCluster::segmentation(){
////    cout << "compressing \n ";
//    cv::Mat tmp;
//    VNode[0]->pointcloud.copyTo(tmp);
////    mCodec->euclideanSegmentation(tmp);
//}

void VCluster::postProcess(){
    if (DECOUPLE2IMG) VNode[0]->mIo->writeCurrentStereoFrame();
#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
    cout << "TimeStamp: " << double(start.tv_sec-tInit.tv_sec)*1000 + double(start.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "postProcess starts" << endl;
#endif
    VNode[0]->analyze();
//    compressDynamic();
//    segmentation();
//    TXRX();
    TXRX_viaDisk();
    visualize();
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "postProcess ends: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
#endif
}



void VCluster::TXRX(){
    if (DEBUG) VNode[0]->mIo->logCurrentFrame();
    // sending objects
    if (TX && SEND) {
//        mSender->writeFullFrame_PC_TCW_Time();
        mSender->writePC_TCW_Time();
    }
    cv::Mat Trc, trc, RxFrame;
    if (RX){
        timeval tTXStart, tTXEnd;
        gettimeofday(&tTXStart, NULL);


        /// receiving objects
        if (!(mReceiver->AskForLatestPC_TCW_TIME(VNode[0]))){
            cerr << "VCluster::TXRX() can't load latest rx frame " << endl;
            return;
        }
        gettimeofday(&tTXEnd, NULL);

        cout << "TXRX >>>>> TX: " <<double(tTXEnd.tv_sec-tTXStart.tv_sec)*1000 + double(tTXEnd.tv_usec-tTXStart.tv_usec) / 1000<< "ms"<< endl;
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
    cv::Mat RxFrame, RxPC, RxTCW, RxDynamicPC;
    if (RX){
        // receiving objects
        // searcing for synced frame
        frameSeqRx ++;
        timeRx = mReceiver->readTimeStamp(frameSeqRx);

        /// time sync module
        unsigned long long int CurrTS = VNode[0]->getCurrentAVRFrame_TimeStamp();



        while( timeRx < CurrTS || timeRx==0){
            frameSeqRx++;
            cout << "reading frame: "  << frameSeqRx;
            cout << "rx ts: " << timeRx << ", cur ts: " << CurrTS << endl;
            timeRx = mReceiver->readTimeStamp(frameSeqRx);
        }

        if (timeRx - CurrTS > 300){
            cout << "current frame lagging behind\n";
            cout << "rx ts: " << timeRx << ", cur ts: " << CurrTS << endl;
            return;
        }

        mReceiver->readPC(frameSeqRx,RxPC);
        if (RxPC.empty()) {
            cerr << "VCluster::TXRX() can't load rx PC " << frameSeqRx << endl;
            return;
        }
        mReceiver->readTcw(frameSeqRx,RxTCW);
        if (RxTCW.empty()) {
            cerr << "VCluster::TXRX() can't load tcw " << frameSeqRx << endl;
            return;
        }
        /// basic info complete
        RxPC.copyTo(VNode[0]->RxPC);
        RxTCW.copyTo(VNode[0]->RxTCW);

        /// not care whether others are atomic fow now
        mReceiver->readFrame(frameSeqRx, VNode[0]->RxFrame);
        if (VNode[0]->RxFrame.empty()){
            cerr << "VCluster::TXRX() can't load rx frame " << frameSeqRx << endl;
            return;
        }
        if (DYNAMICS){
            mReceiver->readDynamicPC(frameSeqRx,VNode[0]->RxDynamicPC);
            mReceiver->readLowPassObjectMotionVec(frameSeqRx, VNode[0]->RxMotionVec);
            if (VNode[0]->RxDynamicPC.empty()){
                cerr << "VCluster::TXRX() can't load rx dynamic frame " << frameSeqRx << endl;
                return;
            }
        }

    }
}

void VCluster::visualize(){
    // need to show PC from Last Frame, cause buffer are freed for pre-fetching
    // Point Cloud Stiching
    if (VISUAL && SHOW_PC ) {
//        mDisplayer->showCurFrame();
        AVRFrame currFrame;
        VNode[0]->getCurrentAVRFrame(currFrame);
        if (TX) {
//                mDisplayer->showPC(VNode[0]->lastStereoData[ZEDCACHESIZE-1].DynamicPC);

            mDisplayer->showPC(currFrame.pointcloud);
//            saveOpenGL(1000, 1000);
        }
        else if (RX) {
            if (VNode[0]->trackGood() && !(VNode[0]->RxTCW.empty())){
                /// calculating rela position
                cv::Mat Trc;
                VNode[0]->calcRelaCamPos(VNode[0]->RxTCW, Trc);

                /// PC manipulation
                VNode[0]->transformRxPCtoMyFrameCoord(Trc, VNode[0]->RxPC, VNode[0]->transRxPC);
                if (DYNAMICS){
                    VNode[0]->transformRxPCtoMyFrameCoord(Trc, VNode[0]->RxDynamicPC, VNode[0]->transRxDynamicPC);
                }
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

                mDisplayer->showMergedPC(VNode[0]->transRxPC);
//                saveOpenGL(VNode[0]->width*2, VNode[0]->height);
//                cv::Mat totalPC, totalDynamicPC;
//                hconcat(VNode[0]->getCurrentAVRFrame().pointcloud, VNode[0]->RxPC, totalPC);
//                hconcat(VNode[0]->pointcloud, transRxDynamicPC, totalDynamicPC);
//                  hconcat(VNode[0]->initPC, VNode[0]->transRxDynamicPC, totalDynamicPC);
//                mDisplayer->showSplitScreen(VNode[0]->getCurrentAVRFrame().pointcloud,totalPC);
//                mDisplayer->showSplitScreen(VNode[0]->pointcloud,totalDynamicPC);
//            mDisplayer->showPC(VNode[0]->transRxDynamicPC);
//                mDisplayer->showPC(VNode[0]->transRxPC);
//                mDisplayer->showPC(totalDynamicPC);

//            mDisplayer->showPC(VNode[0]->transRxPC);
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
            // debug

            mDisplayer->showPC(currFrame.pointcloud);

//            mDisplayer->showPC(VNode[0]->pointcloud_sl_gpu);
        }
    }
}