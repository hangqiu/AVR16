//
// Created by nsl on 9/20/16.
//


#include "Displayer.hpp"
#include "UtilsPC.hpp"
#include "UtilsCV.hpp"
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
    if (RX){

        mGLViewer->init(PCwidth*2,PCheight);
    }else{
        mGLViewer->init(PCwidth,PCheight);
    }

//    PC_gpu.alloc(PCwidth, PCheight, sl::MAT_TYPE_32F_C4, sl::MEM_GPU);
//    PC_gpu.alloc(PCwidth, PCheight, sl::MAT_TYPE_32F_C4, sl::MEM_GPU | sl::MEM_CPU);
//
//  glutMainLoop();
//    display_callback = std::thread(glutMainLoop);
}

Displayer::~Displayer() {
    delete mGLViewer;
//    delete mPC;
//    delete mPCViewer;
}

void Displayer::exit(){
//    PC_gpu.free(MEM_GPU);
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
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    if (currFrame.DynamicFrame.empty()) return;
    char tmp[100];
    sprintf(tmp, "Current Dynamic Left Frame against %d frames before", (idx+1)*5);
    cv::imshow(tmp, currFrame.DynamicFrame);
    char key = cv::waitKey(20);
    processKey(key);
}

void Displayer::showCurFrameWithPC(){
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    if (currFrame.FrameLeft.empty()) return;
    char winName[100] = "Current Left Frame";
    cv::namedWindow(winName);
    cv::imshow(winName, currFrame.FrameLeft);
    cv::setMouseCallback(winName, onMouseCallback_DisplayVoxel, &(currFrame.PC_noColor));
    char key = cv::waitKey(20);
    processKey(key);
}

char Displayer::showCurFrame(){
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    if (currFrame.FrameLeft.empty()) return '0';
    char winName[100] = "Current Left Frame";
//    cv::namedWindow(winName, CV_WINDOW_NORMAL|CV_GUI_NORMAL);
    cv::imshow(winName, currFrame.FrameLeft);
    char key = cv::waitKey(20);
    processKey(key);
    return key;
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
            VNode[0]->mIo->writeCurrentStereoFrame();
            cout << "saved cam 0 as " << tmp_str << endl;
        }
        if (new_key == '1' || new_key == 'a'){
            VNode[1]->mIo->writeCurrentStereoFrame();
            cout << "saved cam 1 as " << tmp_str << endl;
        }
        if (new_key == 'r'){
            cout << "exiting save_img mode\n";break;
        }
    }
}
/////////////////////////////////////// PC Display ///////////////////////////////////////////////


void Displayer::showPC(){
//    checkResetPCViewer(VNode[0]->width, VNode[0]->height);
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    pushPC_cvMat(currFrame.pointcloud);
}

void Displayer::showDynamicPC(){
//    checkResetPCViewer(VNode[0]->width, VNode[0]->height);
    AVRFrame currFrame ;
    VNode[0]->getCurrentAVRFrame(currFrame);
    pushPC_cvMat(currFrame.DynamicPC);
}
void Displayer::pushPC_cvMat(cv::Mat &mat){
    if (DEBUG) debugPC(mat);
    sl::Mat PC_gpu;
    PC_gpu.alloc(PCwidth, PCheight, sl::MAT_TYPE_32F_C4, sl::MEM_CPU);
    cvMat2slMat(mat,PC_gpu, sl::MEM_CPU);
    mGLViewer->updatePointCloud_HostToDevice(PC_gpu);

//    PC_gpu.alloc(PCwidth, PCheight, sl::MAT_TYPE_32F_C4, sl::MEM_GPU);
//    ERROR_CODE  err= PC_gpu.updateGPUfromCPU();
//    if (err!=SUCCESS){
//        cerr << err << endl;
//    }
//    mGLViewer->updatePointCloud(PC_gpu);
//    PC_gpu.free(MEM_GPU);
//    PC_gpu.free(MEM_GPU);
}

void Displayer::showPC(cv::Mat mat) {
    pushPC_cvMat(mat);
}

void Displayer::showPC(sl::Mat& mat) {
    mGLViewer->updatePointCloud(mat);
}

void Displayer::showMergedPC(cv::Mat mat) {
    //TODO check dimension, mat should be the same dimension as my default VNode[0]->pointcloud
//    checkResetPCViewer(mat.cols*2,mat.rows*2);
    cv::Mat totalPC;
    AVRFrame currFrame;
    VNode[0]->getCurrentAVRFrame(currFrame);
    hconcat(currFrame.pointcloud, mat,totalPC);
    pushPC_cvMat(totalPC);
//        VNode[0]->shiftPC(VNode[0]->transformedPointcloud,Scalar(0,-5,0,0));
//        VNode[0]->shiftPC(VNode[0]->DynamicPC,Scalar(0,5,0,0));
//        hconcat(VNode[0]->transformedPointcloud,VNode[0]->DynamicPC,total_point_cloud);
}

void Displayer::showSplitScreen(cv::Mat PC1, cv::Mat PC2){
    //TODO check dimension, mat should be the same dimension as my default VNode[0]->pointcloud
//    checkResetPCViewer(PC1.cols*2,PC1->rows*2);
    cv::Mat totalPC;
//    VNode[0]->shiftPC(PC1,Scalar(0,10,0,0));
//    VNode[0]->shiftPC(PC2,Scalar(0,-10,0,0));
    shiftPC(PC1,Scalar(0,10,0,0));
    shiftPC(PC2,Scalar(0,-10,0,0));
    hconcat(PC1, PC2,totalPC);
    pushPC_cvMat(totalPC);
}