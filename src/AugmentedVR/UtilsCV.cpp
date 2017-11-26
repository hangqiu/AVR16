//
// Created by hang on 9/26/17.
//

#include <opencv2/core/types.hpp>
#include <include/globals.hpp>
#include <GL/gl.h>
#include "UtilsCV.hpp"




void debugCin(){
    char new_key;
    while (true){
        cin >> new_key;
        if (new_key == 'p'){
            PAUSE_FLAG = true;
        }
    }
}

void debugPauser(){
    if (PAUSE_FLAG){
        char new_key;
        cout << "pausing....\n";
        while(new_key != 'r'){
            cin >> new_key;
            if (new_key == 'n') {
                PAUSE_FLAG = true;
                cout << "resuming....\n";
                return;
            }
        }
        cout << "resuming....\n";
        PAUSE_FLAG = false;
    }
}

//
//void pauseStopResume(){
//    if (DEBUG) cout << "pausing....\n";
//    char new_key;
//
//    while(new_key != 'r'){
//        new_key = cv::waitKey(20);
//        if (new_key == 'n') {
//            PAUSE_FLAG = true;
//            if (DEBUG) cout << "resuming....\n";
//            return;
//        }
//    }
//    if (DEBUG) cout << "resuming....\n";
//    PAUSE_FLAG = false;
//}
//void processKey(char key){
//    if (key == 'p' || PAUSE_FLAG) pauseStopResume();
////    if (key == 's') saveFrame();
//}




void onMouseCallback_DisplayDisplacement(int32_t event, int32_t x, int32_t y, int32_t flag, void* param) {

    cv::Mat* PC_dist =(cv::Mat*)param;
    if (event == cv::EVENT_LBUTTONDOWN) {
        cout << "Point: (" << x << "," << y << "), PC: " << (*PC_dist)(cv::Rect(x,y,1,1)) << ", Thresh: "<< MOTIONTHRESH_PERPIXEL*ZEDCACHESIZE <<endl;
    }
}


cv::Mat getGrayBBox(cv::Mat img, int select_left, int select_top, int select_width, int select_height){
    // legalize
    select_left = select_left > 0 ? select_left : 0;
    select_top = select_top > 0 ? select_top : 0;
    select_width = select_width+select_left < img.cols ? select_width : img.cols - select_left;
    select_height = select_height+select_top < img.rows ? select_height : img.rows - select_top;

    cv::Mat img_tmp = img(cv::Rect(select_left, select_top, select_width, select_height));

    cv::cvtColor(img_tmp, img_tmp, CV_BGR2GRAY);
    return img_tmp;
}


//void saveOpenGL(int width, int height){
//    float* buffer = new float[width*height*3];
////    glReadBuffer(GL_BACK);
//    glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, buffer);
//    cv::Mat image(height, width, CV_8UC3, buffer);
//    cv::imshow("Show Image", image);
////    cv::imwrite("Show Image", image);
//}


void detectKLTFeature(cv::Mat & FrameLeftGray, vector<cv::Point2f> & keypoints){
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    cv::Size subPixWinSize(10,10);

    cv::goodFeaturesToTrack(FrameLeftGray, keypoints, MAX_COUNT, 0.01, 10, cv::Mat(), 3, false, 0.04);
    cornerSubPix(FrameLeftGray, keypoints, subPixWinSize, cv::Size(-1,-1), termcrit);
}

void drawMatchedKeypoints(cv::Mat & img, cv::Point2f& kp1, cv::Point2f& kp2, string txtAtKp1){
    circle(img, kp1, 3, cv::Scalar(0, 0, 255), -1, 8);
    circle(img, kp2, 3, cv::Scalar(0, 255, 0), -1, 8);
    line(img, kp1,kp2, cv::Scalar(0, 0, 0));
    cv::putText(img, txtAtKp1, kp1, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
}

void onMouseCallback_DisplayVoxel(int32_t event, int32_t x, int32_t y, int32_t flag, void* param) {

    cv::Mat* PC =(cv::Mat*)param;
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (!(*PC).empty()){
            cout << "Point: (" << x << "," << y << "), PC: " << (*PC)(cv::Rect(x,y,1,1)) << endl;
        }else{
            cout << "Invalid PC\n";
        }
    }
}