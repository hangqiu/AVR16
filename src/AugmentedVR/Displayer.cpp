//
// Created by nsl on 9/20/16.
//


#include "Displayer.hpp"
#include "globals.hpp"
#include "sl/Camera.hpp"
//opencv includes


using namespace cv;
using namespace std;

Displayer::Displayer(AugmentedVR** VNode) : VNode(VNode) {
    mGLViewer = new GLViewer();
    PCwidth = Displayer::VNode[0]->width;
    PCheight = Displayer::VNode[0]->height;
    cout << "Initializing viewer: width, " << PCwidth << ", height, " << PCheight << endl;
    mGLViewer->init(PCwidth,PCheight);
//    glutMainLoop();
//    display_callback = std::thread(glutMainLoop);
}

Displayer::~Displayer() {
    delete mGLViewer;
//    delete mPC;
//    delete mPCViewer;
}

void Displayer::exit(){
    mGLViewer->exit();
}

//void Displayer::init(int argc, char **argv){
//    PCwidth = VNode[0]->width;
//    PCheight = VNode[0]->height;
//    cout << "Initializing viewer: width, " << PCwidth << ", height, " << PCheight << endl;
////    context = VNode[0]->mZEDCam->getCUDAContext();
////    // CREATING POINT CLOUD
//////    mPC = new PointCloud( PCwidth*3, PCheight, context);// it's 2X the size to work around the reallocate bug
////    mPC = new PointCloud( PCwidth, PCheight, context);// it's 2X the size to work around the reallocate bug
////    // the receiver's GPU context
////
////    //Create windows for viewing results with OpenCV
////    mPCViewer = new Viewer(*mPC, argc, argv);
//
//    mGLViewer->init(PCwidth/2,PCheight/2);
//
//    std::thread display_callback = std::thread(glutMainLoop);
//    //wait for OpenGL stuff to be initialized
////    if (VISUAL || SHOW_PC) while (!mPCViewer->isInitialized() ); //&& !single_viewer.isInitialized()
//}


void onMouseCallback_DisplayVoxel(int32_t event, int32_t x, int32_t y, int32_t flag, void* param) {

    cv::Mat* PC =(cv::Mat*)param;
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (!(*PC).empty()){
            cout << "Point: (" << x << "," << y << "), PC: " << (*PC)(Rect(x,y,1,1)) << endl;
        }else{
            cout << "Invalid PC\n";
        }
    }
}

void Displayer::showCurDynamicFrame(int idx){
    if (VNode[0]->lastStereoData[ZEDCACHESIZE-1-idx].DynamicFrame.empty()) return;
    char tmp[100];
    sprintf(tmp, "Current Dynamic Left Frame against %d frames before", (idx+1)*5);
    cv::imshow(tmp, VNode[0]->lastStereoData[ZEDCACHESIZE-1-idx].DynamicFrame);
    char key = cv::waitKey(20);
    processKey(key);
}

void Displayer::showCurFrame(){
    if (VNode[0]->FrameLeft.empty()) return;
    char winName[100] = "Current Left Frame";
    cv::namedWindow(winName);
    cv::imshow(winName, VNode[0]->FrameLeft);
    cv::setMouseCallback(winName, onMouseCallback_DisplayVoxel, &(VNode[0]->PC_noColor));
    char key = cv::waitKey(20);
    processKey(key);
}


void Displayer::showImgWithPC(cv::Mat& img, cv::Mat* PC, char * winName){
//    if (VNode[0]->FrameLeft.empty()) return;
//    char winName[100] = "Current Left Frame";
    cv::namedWindow(winName);
    cv::imshow(winName, img);
    cv::setMouseCallback(winName, onMouseCallback_DisplayVoxel, PC);
    char key = cv::waitKey(20);
    processKey(key);
}

void Displayer::processKey(char key){
    if (key == 'p' || PAUSE_FLAG) pauseStopResume();
    if (key == 's') saveFrame();
}

void Displayer::pauseStopResume(){
    if (DEBUG) cout << "pausing....\n";
    char new_key;

    while(new_key != 'r'){
        new_key = cv::waitKey(20);
        if (new_key == 'n') {
            PAUSE_FLAG = true;
            if (DEBUG) cout << "resuming....\n";
            return;
        }
        if (new_key == 's') {
            saveFrame();
        }
    }
    if (DEBUG) cout << "resuming....\n";
    PAUSE_FLAG = false;
}

void Displayer::saveFrame(){
    char tmp_str[50];
    cout << "press the number of camera you want to save, a for all\n";
    char new_key;
    while(new_key = cv::waitKey(20)){
        if (new_key == '0' || new_key == 'a'){
            VNode[0]->saveCurFrame();
            cout << "saved cam 0 as " << tmp_str << endl;
        }
        if (new_key == '1' || new_key == 'a'){
            VNode[1]->saveCurFrame();
            cout << "saved cam 1 as " << tmp_str << endl;
        }
        if (new_key == 'r'){
            cout << "exiting save_img mode\n";break;
        }
    }
}
/////////////////////////////////////// PC Display ///////////////////////////////////////////////
void Displayer::debugPC(cv::Mat DebugPC){
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
}

void Displayer::showPC(){
//    checkResetPCViewer(VNode[0]->width, VNode[0]->height);
    pushPC_Mat(VNode[0]->pointcloud_cv);
}

void Displayer::showDynamicPC(){
//    checkResetPCViewer(VNode[0]->width, VNode[0]->height);
    pushPC_Mat(VNode[0]->lastStereoData[ZEDCACHESIZE-1].DynamicPC);
}
void Displayer::pushPC_Mat(cv::Mat& mat){
    if (DEBUG>1) debugPC(mat);
    sl::Mat PC_gpu;
    cvMat2slMat(mat,PC_gpu);

    PC_gpu.alloc(mat.size().width, mat.size().height, sl::MAT_TYPE_8U_C4, sl::MEM_GPU);

    ERROR_CODE  err= PC_gpu.updateGPUfromCPU();
    if (err!=SUCCESS){
        cerr << err << endl;
    }
    mGLViewer->updatePointCloud(PC_gpu);
    PC_gpu.free(MEM_GPU);
    PC_gpu.free(MEM_GPU);
    // show the point cloud
//    if (mPC->mutexData.try_lock()) {
//        mPC->pushNewPC_HostToDevice(PC_gpu);
//        mPC->mutexData.unlock();
//    }
}

void Displayer::showPC(cv::Mat mat) {
    pushPC_Mat(mat);
}

void Displayer::showPC(sl::Mat& mat) {
    mGLViewer->updatePointCloud(mat);
}

void Displayer::showMergedPC(cv::Mat mat) {
    //TODO check dimension, mat should be the same dimension as my default VNode[0]->pointcloud
//    checkResetPCViewer(mat.cols*2,mat.rows*2);
    cv::Mat totalPC;
    hconcat(VNode[0]->pointcloud_cv, mat,totalPC);
    pushPC_Mat(totalPC);
//        VNode[0]->shiftPC(VNode[0]->transformedPointcloud,Scalar(0,-5,0,0));
//        VNode[0]->shiftPC(VNode[0]->DynamicPC,Scalar(0,5,0,0));
//        hconcat(VNode[0]->transformedPointcloud,VNode[0]->DynamicPC,total_point_cloud);
}

void Displayer::showSplitScreen(cv::Mat PC1, cv::Mat PC2){
    //TODO check dimension, mat should be the same dimension as my default VNode[0]->pointcloud
//    checkResetPCViewer(PC1.cols*2,PC1.rows*2);
    cv::Mat totalPC;
    VNode[0]->shiftPC(PC1,Scalar(0,10,0,0));
    VNode[0]->shiftPC(PC2,Scalar(0,-10,0,0));
    hconcat(PC1, PC2,totalPC);
    pushPC_Mat(totalPC);
}