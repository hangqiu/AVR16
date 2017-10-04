//
// Created by hang on 9/26/17.
//

#include <opencv2/core/types.hpp>
#include <include/globals.hpp>
#include "CVUtils.hpp"



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






