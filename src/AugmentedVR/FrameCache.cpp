//
// Created by hang on 9/26/17.
//

#include <sl/Camera.hpp>
#include <include/globals.hpp>
#include <cv.hpp>
#include <include/UtilsPC.hpp>
#include <sys/time.h>
#include "FrameCache.hpp"

using namespace sl;
using namespace std;

FrameCache::FrameCache(){
    CacheSize = ZEDCACHESIZE;
    assert(CacheSize>4); // must > 4. since simultanesouly processing 4 frames, 3 + ZEDCACHESIZE ensure comparing ZEDCACHESIZE frames back
//    NextFrame = AVRFrame();
//    SlamFrame = AVRFrame();
//    CurrentFrame = AVRFrame();
//    LastFrame = AVRFrame();
    startTS = 0;
    fifoStartIndex = -1;
    fifoEndIndex = -1;
    fifo = new  AVRFrame[CacheSize];
}
FrameCache::FrameCache(int size){
    CacheSize = size;
    assert(CacheSize>4); // must > 4. since simultanesouly processing 4 frames, 3 + ZEDCACHESIZE ensure comparing ZEDCACHESIZE frames back
//    NextFrame = AVRFrame();
//    SlamFrame = AVRFrame();
//    CurrentFrame = AVRFrame();
//    LastFrame = AVRFrame();
    startTS = 0;
    fifoStartIndex = -1;
    fifoEndIndex = -1;
    fifo = new AVRFrame[CacheSize];
}

FrameCache::~FrameCache(){
//    delete[] fifo;
}


void FrameCache::GrabNewZEDFrame(AVRFrame& NewFrame, Camera* mZEDCam, int width, int height, int seq){

#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
//    cout << "grabZEDFrameOffline >>>>>>> PC thread starting at: " << double(start.tv_sec)*1000 + double(start.tv_usec) / 1000<< "ms" << endl;
#endif
    NewFrame.FrameLock.lock();
    NewFrame.ZEDTS = mZEDCam->getCameraTimestamp();
    LatestZEDTS = NewFrame.ZEDTS;
    NewFrame.frameTS = (NewFrame.ZEDTS - startTS)/1000;
    LatestFrameTS = NewFrame.frameTS;
    NewFrame.frameSeq = seq;
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "grabZEDFrameOffline >>>>>>>  Grab: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
    gettimeofday(&start, NULL);
#endif
    // SDK2.0
    sl::Mat pc;
    ERROR_CODE err = mZEDCam->retrieveMeasure(pc, sl::MEASURE_XYZRGBA, sl::MEM_CPU, width, height);
    if (err!=SUCCESS)        cerr << "Can't retrieve point cloud! error code:" << err << endl;

    slMat2cvMat(pc,NewFrame.pointcloud); //just to be safe
//    ERROR_CODE err2 = mZEDCam->retrieveMeasure(pointcloud_sl_gpu, sl::MEASURE_XYZRGBA, sl::MEM_GPU, width, height);
//    if (err2!=SUCCESS){
//        cerr << "Can't retrieve point cloud gpu! error code:" << err2 << endl;
//    }
    stripPointCloudColorChannel(NewFrame.pointcloud,NewFrame.PC_noColor);

#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "grabZEDFrameOffline >>>>>>>   PC in CPU: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
    gettimeofday(&start, NULL);
#endif
    // SDK 2.0
    sl::Mat frameLeft_cpu, frameRight_cpu;
    mZEDCam->retrieveImage(frameLeft_cpu, sl::VIEW_LEFT,sl::MEM_CPU);
    mZEDCam->retrieveImage(frameRight_cpu, sl::VIEW_RIGHT,sl::MEM_CPU);
    slMat2cvMat(frameLeft_cpu, NewFrame.FrameLeft);
    slMat2cvMat(frameRight_cpu, NewFrame.FrameRight);
//    cv::imshow("newframe",NewFrame.FrameLeft);


    cv::cvtColor(NewFrame.FrameLeft, NewFrame.FrameLeftGray, CV_BGR2GRAY);
    cv::cvtColor(NewFrame.FrameRight, NewFrame.FrameRightGray, CV_BGR2GRAY);

    NewFrame.FrameLock.unlock();
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "grabZEDFrameOffline >>>>>>> Frame: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000<< "ms" << endl;
//        cout << ">>>>>>> PC thread ending at: " << double(end.tv_sec)*1000 + double(end.tv_usec) / 1000<< "ms"<< endl;
#endif
}

void FrameCache::LoadNextFrameFromZED(sl::Camera *mZEDCam, int width, int height, int seq){

    GrabNewZEDFrame(NextFrame, mZEDCam,width,height,seq);

}



//bool FrameCache::FIFOTail2NextFrame(){
//    if (fifo.size()==0) return false;
////    assert(NextFrame==NULL);
////    NextFrame = new AVRFrame(fifo[fifo.size()-1]);
//    NextFrame.setFrom(fifo[fifo.size()-1]); // point to the tail of the fifo.
//    return true;
//}

void FrameCache::SinkFrames(){
    theBigLock.lock();
    if (!LastFrame2FIFO()) {
        if (DEBUG) cerr<< "FrameCache::SinkFrames LastFrame2FIFO Failure\n";
    }
    if (!CurrentFrame2LastFrame()) {
        if (DEBUG) cerr<< "FrameCache::SinkFrames CurrentFrame2LastFrame Failure\n";
    }
    if (!SlamFrame2CurrentFrame()) {
        if (DEBUG) cerr<< "FrameCache::SinkFrames SlamFrame2CurrentFrame Failure\n";
    }
    if (!NextFrame2SlamFrame()) {
        if (DEBUG) cerr<< "FrameCache::SinkFrames NextFrame2SlamFrame Failure\n";
    }
    theBigLock.unlock();
}

bool FrameCache::NextFrame2SlamFrame(){
    if (NextFrame.isEmpty()) return false;
    SlamFrame.setFrom(NextFrame);
//    NextFrame.setFrom(AVRFrame());
    return true;
}

bool FrameCache::SlamFrame2CurrentFrame(){
    if (SlamFrame.isEmpty()) return false;
    CurrentFrame.setFrom(SlamFrame);
//    SlamFrame.setFrom(AVRFrame());
    return true;
}

bool FrameCache::CurrentFrame2LastFrame(){
    if (CurrentFrame.isEmpty()) return false;
    LastFrame.setFrom(CurrentFrame);
//    CurrentFrame.setFrom(AVRFrame());
    return true;
}

void FrameCache::getFIFOHead(AVRFrame& frame){
    frame.setFrom(fifo[fifoStartIndex]);
}

bool FrameCache::LastFrame2FIFO(){
    if (LastFrame.isEmpty()) return false;
    fifoEndIndex++;
    if (fifoEndIndex == CacheSize){
        fullBacklog = true;
    }
    fifoEndIndex%=CacheSize;
    fifo[fifoEndIndex].setFrom(LastFrame);
    if (fifoEndIndex==fifoStartIndex || fifoStartIndex==-1){
        fifoStartIndex++;
        fifoStartIndex%=CacheSize;
    }

    return true;
}
//
//bool FrameCache::LastFrame2FIFO(){
//    if (LastFrame.isEmpty()) return false;
//    AVRFrame* newFrame = new AVRFrame();
//    newFrame->setFrom(LastFrame);
//    if (fifo.size()==CacheSize){
////        vector<AVRFrame>* tmp = new vector<AVRFrame>;
////        for (int i=1;i<CacheSize;i++){
////            tmp->push_back(fifo[i]);
////        }
////        fifo.swap(*tmp);
////        for (int i=0;i<tmp->size();i++){
////            delete &(tmp[i]);
////        }
////        tmp->clear();
////        delete tmp;
//
////        delete &(fifo[0]);
////        AVRFrame* ptr = &fifo[0];
//        fifo.erase(fifo.begin());
//        fifo.resize(fifo.size());
//        fifo.shrink_to_fit();
////        delete ptr;
//    }
//    fifo.push_back(*newFrame);
//
//    return true;
//}
//



void FrameCache::opticalFlowTrack_Curr2Last(){
    theBigLock.lock();
    LastFrame.FrameLock.lock();
    CurrentFrame.FrameLock.lock();
    calcOpticalFlow_Current2Last();
    FindHomographyMatrix_Curr2Last();
    CurrentFrame.FrameLock.unlock();
    LastFrame.FrameLock.unlock();
    theBigLock.unlock();
}

bool FrameCache::calcOpticalFlow_Current2Last() {


    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size  winSize(31, 31);
#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
#endif
    if (LastFrame.isEmpty()) {
        cerr << "FrameCache::calcOpticalFlow_Current2Last(): Not enough history frames\n";
        return true;
    } else if (LastFrame.keypoints.empty()) {
        cerr << "FrameCache::calcOpticalFlow_Current2Last(): Last frame has no keypoints yet\n";
        return true;
    }

    // 2D track tracked points
    cv::calcOpticalFlowPyrLK(LastFrame.FrameLeftGray, CurrentFrame.FrameLeftGray,
                             LastFrame.keypoints, CurrentFrame.keypoints,
                             CurrentFrame.status, CurrentFrame.error, winSize, 3, termcrit, 0, 0.001);
    // Feature quality is stored with the frame
    if (!LastFrame.status.empty()) {
        for (int i = 0; i < CurrentFrame.status.size(); i++) {
            CurrentFrame.status[i] &= LastFrame.status[i];
        }
    }

    if (LastFrame.tracked_keypoints.empty()) {
        cerr << "FrameCache::calcOpticalFlow_Current2Last(): Last frame has no tracked keypoints yet\n";
        return true;
    }
    // 2D track current points
    cv::calcOpticalFlowPyrLK(LastFrame.FrameLeftGray, CurrentFrame.FrameLeftGray,
                             LastFrame.tracked_keypoints, CurrentFrame.tracked_keypoints,
                             CurrentFrame.tracked_status, CurrentFrame.tracked_error, winSize, 3, termcrit, 0, 0.001);
    if (!LastFrame.tracked_status.empty()) {
        for (int i = 0; i < CurrentFrame.tracked_status.size(); i++) {
            CurrentFrame.tracked_status[i] &= LastFrame.tracked_status[i];
        }
    }
#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "fetchNUpdateFrameNPointcloud >>>>>> OF: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
#endif
    return true;
}


void FrameCache::FindHomographyMatrix_Curr2Last(){
#ifdef EVAL
    timeval start, end;
    gettimeofday(&start, NULL);
#endif

    // FIND HOMOGRAPHY MATRIX
    //-- Localize the object
    std::vector<cv::Point2f> good_lastKeypoints, good_Keypoints;
    std::vector<cv::Point2f> obj_corners(4);
    std::vector<cv::Point2f> scene_corners(4);

    for( size_t i = 0; i < CurrentFrame.keypoints.size(); i++ ){
        if (CurrentFrame.status[i]){
            good_lastKeypoints.push_back(LastFrame.keypoints[i]);
            good_Keypoints.push_back(CurrentFrame.keypoints[i]);
        }
    }
    cv::Mat H;
    if (!good_Keypoints.empty())
        H = findHomography( good_Keypoints,good_lastKeypoints, cv::RANSAC );

    H.copyTo(CurrentFrame.sceneTransformMat);
    //    cout << "Homography matrix: " << H << endl




#ifdef EVAL
    gettimeofday(&end, NULL);
    cout << "TimeStamp: " << double(end.tv_sec-tInit.tv_sec)*1000 + double(end.tv_usec-tInit.tv_usec) / 1000 << "ms: ";
    cout << "fetchNUpdateFrameNPointcloud>>>>>>Homography: " << double(end.tv_sec-start.tv_sec)*1000 + double(end.tv_usec-start.tv_usec) / 1000 << "ms" << endl;
#endif
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    // debug
    //-- Get the corners from the image_1 ( the object to be "detected" )
    int margin=50;
    obj_corners[0] = cvPoint(margin,margin);
    obj_corners[1] = cvPoint( CurrentFrame.FrameLeft.cols-margin, margin );
    obj_corners[2] = cvPoint( CurrentFrame.FrameLeft.cols-margin, CurrentFrame.FrameLeft.rows-margin );
    obj_corners[3] = cvPoint( margin, CurrentFrame.FrameLeft.rows-margin );

    if (DEBUG && VISUAL){
        cv::Mat img, img_cache;
        CurrentFrame.FrameLeft.copyTo(img);LastFrame.FrameLeft.copyTo(img_cache);
        int count = 0;

        for( size_t i = 0; i < CurrentFrame.keypoints.size(); i++ ){
            if (CurrentFrame.status[i]){
                count ++;
                circle( img_cache, LastFrame.keypoints[i], 3, cv::Scalar(255,0,0), -1, 8);
                circle( img_cache, CurrentFrame.keypoints[i], 3, cv::Scalar(0,255,0), -1, 8);
                line( img_cache, LastFrame.keypoints[i], CurrentFrame.keypoints[i], cv::Scalar(0,0,255));
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
}

//int FrameCache::getCacheSize() const {
//    return CacheSize;
//}

void FrameCache::updateMotionData_Curr2CacheHead(){
    assert(ReachFullMotionBacklog());

    CurrentFrame.FrameLock.lock();
    fifo[fifoStartIndex].FrameLock.lock();
    //PC motion
    updateTransformationMatrix_Curr2CacheHead();
    CurrentFrame.tranformPointCloud_via_TransformationMat();
    updateMotionVec_Curr2CacheHead();
    // filter dynamics
    CurrentFrame.MotionAnalysis();

    fifo[fifoStartIndex].FrameLock.unlock();
    CurrentFrame.FrameLock.unlock();
}

void FrameCache::updateMotionVec_Curr2CacheHead(){
    CurrentFrame.PCMotionVec = CurrentFrame.transformedPointcloud - fifo[fifoStartIndex].pointcloud;
}

void FrameCache::updateTransformationMatrix_Curr2CacheHead(){
    // w: world, l: last frame, c: current frame
    assert(ReachFullMotionBacklog());
    if (fifo[fifoStartIndex].CamMotionMat.empty() || CurrentFrame.CamMotionMat.empty()) {
        cerr << "FrameCache::updateTransformationMatrix_Curr2CacheHead: No CamMotionMat\n";
        return;
    }
    const cv::Mat Rcw = CurrentFrame.CamMotionMat.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.CamMotionMat.rowRange(0,3).col(3);
    const cv::Mat Rwc = Rcw.t();
    const cv::Mat twc = -Rwc*tcw;

    const cv::Mat Rlw = fifo[fifoStartIndex].CamMotionMat.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = fifo[fifoStartIndex].CamMotionMat.rowRange(0,3).col(3);

    CurrentFrame.TranslationMat_Curr2CacheHead= Rlw*twc+tlw;
    CurrentFrame.RotationMat_Curr2CacheHead = Rlw*Rwc;
    if (DEBUG>1){
        cout << "tlc: \n" << CurrentFrame.TranslationMat_Curr2CacheHead << endl;
        cout << "Rlc: \n" << CurrentFrame.RotationMat_Curr2CacheHead << endl; // to see if Rlc is almost identity matrix
    }
}




bool FrameCache::ReachFullMotionBacklog(){
//     return fifo.size()==CacheSize && !(fifo[0].CamMotionMat.empty());
    return fullBacklog && !(fifo[fifoStartIndex].CamMotionMat.empty()) && !(CurrentFrame.CamMotionMat.empty());
}

void FrameCache::setStartTS(unsigned long long int startTS) {
    FrameCache::startTS = startTS;
}


void  FrameCache::getCurrentFrame(AVRFrame & ret){
    theBigLock.lock();
    ret.setFrom(CurrentFrame);
    theBigLock.unlock();
}

void  FrameCache::getCurrentFrame_PointCloud(cv::Mat & ret){
    theBigLock.lock();
    CurrentFrame.getPointCloud(ret);
    theBigLock.unlock();
}



void  FrameCache::getNextFrame(AVRFrame & ret){
    theBigLock.lock();
    ret.setFrom(NextFrame);
    theBigLock.unlock();
}
void  FrameCache::getSlamFrame(AVRFrame & ret){
    theBigLock.lock();
    ret.setFrom(SlamFrame);
    theBigLock.unlock();
}
void  FrameCache::getLastFrame(AVRFrame & ret){
    theBigLock.lock();
    ret.setFrom(LastFrame);
    theBigLock.unlock();
}

void FrameCache::updateCurrFrameFeature(){
    theBigLock.lock();
    CurrentFrame.updateFeature();
    theBigLock.unlock();
}

void FrameCache::updateLastFrameFeature(){
    theBigLock.lock();
    LastFrame.updateFeature();
    theBigLock.unlock();
}

unsigned long long int FrameCache::getLatestZEDTS()  {
    unsigned long long int ret;
    theBigLock.lock();
    ret = LatestZEDTS;
    theBigLock.unlock();
    return ret;
}

unsigned long long int FrameCache::getLatestFrameTS()  {
    unsigned long long int ret;
    theBigLock.lock();
    ret = LatestFrameTS;
    theBigLock.unlock();
    return ret;
}
