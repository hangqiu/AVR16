
#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <ctype.h>
#include <float.h>
#include <stdlib.h>
#include <cmath>

#include "DeepCam/CMT/CMT.h"
#include "DeepCam/CMT/gui.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

// #include <opencv2/nonfree/nonfree.hpp>

//hang add color layout descriptors
#include "DeepCam/MPEG7_Lib/Feature.h"
#include "DeepCam/MPEG7_Lib/FexWrite.h"
#include "DeepCam/MPEG7_Lib/Frame.h"

//for metadata
#include "DeepCam/include/medusa.h"
#include "DeepCam/include/Object.h"
#include "obj_record.pb.h"

#include "DeepCam/include/DeepCam.h"
//logging
#include "DeepCam/CMT/logging/log.h"

#include "DeepCam/include/globals.h"
#include "DeepCam/include/MultiObjTracker.h"

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;
using namespace cv;
using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

#include "DeepCam/YOLO/detection.h"

#ifdef __cplusplus
}
#endif
//extern int meta_id;
//extern struct meta_struct meta;




//Mat im, im_gray, im_0, im_0_gray, im0, curGray;
//cv::Mat prevGray, prevIm;
//vector<CMT> cmt_vec;
//vector<Object> existing_obj;
//vector<Object> moved_obj;
//const int MAX_COUNT = 500;

//vector<Point2f> points[2];
//std::vector<struct LKT_IDX> lkt_obj; // index must match with existing_obj

//long ts;
//int fps;
//double azimuth;
//double lat, lon;

bool end_flag = false;
bool write_first_flag = true;
//bool debug_flag = true;
//char rec_frame[100];
//int keypt_num=0;



//vector<uchar> status;
//vector<float> err;

//struct LKT_IDX{
//    vector<int> idx_vec;
//};






//vector<float> getNextLineAndSplitIntoFloats(istream& str)
//{
//    vector<float>   result;
//    string                line;
//    getline(str,line);
//
//    stringstream          lineStream(line);
//    string                cell;
//    while(getline(lineStream,cell,','))
//    {
//        result.push_back(atof(cell.c_str()));
//    }
//    return result;
//}

//int display(Mat im, CMT & cmt)
//{
//    //Visualize the output
//    //It is ok to draw on im itself, as CMT only uses the grayscale image
//    for(size_t i = 0; i < cmt.points_active.size(); i++)
//    {
//        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
//    }
//
//    Point2f vertices[4];
//    cmt.bb_rot.points(vertices);
//    for (int i = 0; i < 4; i++)
//    {
//        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
//    }
//
//    if (SHOW_IMG) {imshow(WIN_NAME, im);}
//    if (WRITE_VIDEO)  outputVideo<<im;
//    return waitKey(5);
//}

//char display_multi(Mat im, vector<CMT> & cmt_vec, vector<Object> existing_obj)
//{
//    //Visualize the output
//    //It is ok to draw on im itself, as CMT only uses the grayscale image
//    const bool points = true;
//    Point2f vertices[4];
//    for (int rect_idx=0;rect_idx < int(cmt_vec.size());rect_idx++){
//
//        if (points){
//            for(size_t i = 0; i < cmt_vec[rect_idx].points_active.size(); i++)
//            {
//                cv::circle(im, cmt_vec[rect_idx].points_active[i], 2, Scalar(255,0,0));
//            }
//        }
//        cmt_vec[rect_idx].bb_rot.points(vertices);
//        for (int i = 0; i < 4; i++)
//        {
//            if (existing_obj[rect_idx].alive){
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
//            }else{
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(0,0,255));
//            }
//        }
//        // vertices[0].x = existing_obj[rect_idx].box.left;
//        // vertices[1].x = existing_obj[rect_idx].box.left;
//        // vertices[2].x = existing_obj[rect_idx].box.right;
//        // vertices[3].x = existing_obj[rect_idx].box.right;
//        // vertices[0].y = existing_obj[rect_idx].box.bot;
//        // vertices[1].y = existing_obj[rect_idx].box.top;
//        // vertices[2].y = existing_obj[rect_idx].box.top;
//        // vertices[3].y = existing_obj[rect_idx].box.bot;
//        // for (int i = 0; i < 4; i++)
//        // {
//        //     line(im, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
//        // }
//        char tmp[30];
//        sprintf(tmp,"%s%d", existing_obj[rect_idx].box.type, existing_obj[rect_idx].id);
//        putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//        // putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//        // putText(im, obj_bb_box[rect_idx].score, vertices[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//
//    }
//
//    if (SHOW_IMG) {imshow(WIN_NAME, im);}
//    if (WRITE_VIDEO){ outputVideo<<im;}
//
//    return waitKey(5);
//}
//
//int display_2_obj(Mat im, vector<Object> existing_obj, vector<Object> moved_obj)
//{
//    Point2f vertices[4];
//    for (int rect_idx=0;rect_idx < int(existing_obj.size());rect_idx++){
//        if (existing_obj[rect_idx].alive){
//
//            vertices[0].x = existing_obj[rect_idx].box.left;
//            vertices[1].x = existing_obj[rect_idx].box.left;
//            vertices[2].x = existing_obj[rect_idx].box.right;
//            vertices[3].x = existing_obj[rect_idx].box.right;
//            vertices[0].y = existing_obj[rect_idx].box.bot;
//            vertices[1].y = existing_obj[rect_idx].box.top;
//            vertices[2].y = existing_obj[rect_idx].box.top;
//            vertices[3].y = existing_obj[rect_idx].box.bot;
//            for (int i = 0; i < 4; i++)
//            {
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
//            }
//            char tmp[30];
//            sprintf(tmp,"%s%d", existing_obj[rect_idx].box.type, existing_obj[rect_idx].id);
//            putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//            vertices[0].x = moved_obj[rect_idx].box.left;
//            vertices[1].x = moved_obj[rect_idx].box.left;
//            vertices[2].x = moved_obj[rect_idx].box.right;
//            vertices[3].x = moved_obj[rect_idx].box.right;
//            vertices[0].y = moved_obj[rect_idx].box.bot;
//            vertices[1].y = moved_obj[rect_idx].box.top;
//            vertices[2].y = moved_obj[rect_idx].box.top;
//            vertices[3].y = moved_obj[rect_idx].box.bot;
//            for (int i = 0; i < 4; i++)
//            {
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
//            }
//            // char tmp[30];
//            // sprintf(tmp,"%s%d", existing_obj[rect_idx].box.type, existing_obj[rect_idx].id);
//            // putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//        }
//        // putText(im, obj_bb_box[rect_idx].score, vertices[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//
//    }
//
//    if (SHOW_IMG) {imshow(WIN_NAME, im);}
//    if (WRITE_VIDEO){ outputVideo<<im;}
//
//    return waitKey(1000);
//}
//
//char displayExistingObjOnPrevIm(Mat im, vector<Object> existing_obj, int R, int G, int B)
//{
//    Point2f vertices[4];
//    for (int rect_idx=0;rect_idx < int(existing_obj.size());rect_idx++){
//        if (existing_obj[rect_idx].alive){
//
//            vertices[0].x = existing_obj[rect_idx].box.left;
//            vertices[1].x = existing_obj[rect_idx].box.left;
//            vertices[2].x = existing_obj[rect_idx].box.right;
//            vertices[3].x = existing_obj[rect_idx].box.right;
//            vertices[0].y = existing_obj[rect_idx].box.bot;
//            vertices[1].y = existing_obj[rect_idx].box.top;
//            vertices[2].y = existing_obj[rect_idx].box.top;
//            vertices[3].y = existing_obj[rect_idx].box.bot;
//            for (int i = 0; i < 4; i++)
//            {
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(R,G,B),3);
//            }
//            char tmp[30];
//            sprintf(tmp,"%s%d:%s", existing_obj[rect_idx].box.type, existing_obj[rect_idx].id,existing_obj[rect_idx].status.c_str());
//            // sprintf(tmp,"%s", existing_obj[rect_idx].box.type);
//            putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//            if (DEBUG) printf("displaying box: %d,%d,%d,%d\n", existing_obj[rect_idx].box.left,existing_obj[rect_idx].box.right,existing_obj[rect_idx].box.top,existing_obj[rect_idx].box.bot);
//        }
//        // putText(im, obj_bb_box[rect_idx].score, vertices[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//    }
//
//    if (SHOW_IMG) {imshow(WIN_NAME, im);}
//    if (WRITE_VIDEO){ outputVideo<<im;}
//
//    return waitKey(5);
//}

//vector<Object> get_obj_box_from_cmt_vec(vector<CMT> & cmt_vec, vector<Object> & existing_obj){
//    Point2f vertices[4];
//    vector<Object> moved_obj = existing_obj;
//    for (int rect_idx=0;rect_idx < int(cmt_vec.size());rect_idx++){
//        cmt_vec[rect_idx].bb_rot.points(vertices);
////         obj_box tmp_box;
//        // Object tmp_obj;
//
//        // tmp_obj.box.left = vertices[0].x;
//        // tmp_obj.box.bot = vertices[0].y;
//        // tmp_obj.box.right = vertices[2].x;
//        // tmp_obj.box.top = vertices[2].y;
//        // tmp_obj.id = existing_obj[rect_idx].id;
//        // tmp_obj.tracked = false;
//        // moved_obj.push_back(tmp_obj);
//
//        moved_obj[rect_idx].box.left = vertices[0].x;
//        moved_obj[rect_idx].box.bot = vertices[0].y;
//        moved_obj[rect_idx].box.right = vertices[2].x;
//        moved_obj[rect_idx].box.top = vertices[2].y;
//        // moved_obj[rect_idx].id = existing_obj[rect_idx].id;
//        moved_obj[rect_idx].tracked = false;
//        // moved_obj.push_back(tmp_obj);
//        // printf("Box From CMT: left,%d,top,%d,right,%d,bot,%d\n", tmp_box.left,tmp_box.top,tmp_box.right,tmp_box.bot );
//    }
//    return moved_obj;
//}






//hang added function
// get color layout
//
//void writeCLD(Mat img, char* filepath){
//    Frame* frame = new Frame(img.cols, img.rows, true, true, true);
//    // set the image of the frame
//    frame->setImage(img);
//    // CLD with numberOfYCoeff (28), numberOfCCoeff (15)
//    // int start = cvGetTickCount();
//    FexWrite::computeWriteCLD( frame, 28, 15 , filepath );
//    // int mid = cvGetTickCount();
//    // double detect_time = (double)(mid-start)/ (cvGetTickFrequency()) / 1000000;
//    // cout << endl << detect_time << endl;
//    // release frame
//    delete frame;
//    return;
//}


Rect zoom_rect(Rect rect, double scale){
    int new_x, new_y, new_width, new_height;
    new_x = rect.x + rect.height / 2 - int(double(rect.height) * scale / 2);
    new_y = rect.y + rect.width / 2 - int(double(rect.width) * scale / 2);
    new_width = int(double(rect.width)*scale);
    new_height = int(double(rect.height)*scale);
    if (DEBUG) printf("old:%d,%d,%d,%d\n",rect.x,rect.y,rect.width,rect.height );
    if (DEBUG) printf("new:%d,%d,%d,%d\n",new_x,new_y,new_width,new_height );
    Rect new_rect = Rect(new_x,new_y,new_width,new_height);
    return new_rect;
}

//// modify the bounding box to be within the frame
//Rect legalize_box(Mat im0, Rect bbox){
//    // printf("Oringal: %d,%d,%d,%d\n", rect_tmp.x,rect_tmp.y,rect_tmp.x+rect_tmp.width,rect_tmp.y+rect_tmp.height );
//    if (bbox.x<0){
//        bbox.width += bbox.x;
//        bbox.x = 0;
//    }
//    if (bbox.y<0){
//        bbox.height += bbox.y;
//        bbox.y = 0;
//    }
//    if (bbox.y+bbox.height > im0.rows){
//        bbox.height =  im0.rows - bbox.y;
//    }
//    if (bbox.x+bbox.width > im0.cols){
//        bbox.width = im0.cols- bbox.x;
//    }
//    return bbox;
//    // printf("Legalized: %d,%d,%d,%d\n", rect_tmp.x,rect_tmp.y,rect_tmp.x+rect_tmp.width,rect_tmp.y+rect_tmp.height );
//}




//////////////////////////////////////////////////////////// Preview Stage/////////////////////////////////////////////////
//Show preview until key is pressed
//void show_preview(double scale, VideoCapture cap, ifstream & metadata_file){
//    Mat preview_0,preview;
//    while (true)
//    {
//        preview_0 = load_frame_dump_metadata(cap,metadata_file);
//        if (preview_0.empty()) {
//            cout<<"End of video\n";
//            return;
//        }
//        preview = resize_Input_color(preview_0,scale);
//
//        screenLog(preview, "Press a key to start tracking an object.");
//        if (SHOW_IMG){imshow(WIN_NAME, preview);}
//        char k = waitKey(33);
//        if (k == 'r') {
//            break;
//        }
//    }
//}



// test if it is overlapping with another bb_box
// bool overlap_test(int idx, vector<Rect> Rects, vector<struct obj_box> obj_bb_box, vector<char *> Types ){

//     int sub_idx=0;
//     int overlapping=0;
//     int max_top,max_left,min_bot,min_right;
//     double ratio1, ratio2;
//     bool overlap_flag = false;
//     for (sub_idx=idx-1;sub_idx>=0;sub_idx--){
//         max_left = max(Rects[sub_idx].x,obj_bb_box[idx].left);
//         min_right = min(Rects[sub_idx].x+Rects[sub_idx].width,obj_bb_box[idx].right);
//         max_top = max(Rects[sub_idx].y,obj_bb_box[idx].top);
//         min_bot = min(Rects[sub_idx].y+Rects[sub_idx].height,obj_bb_box[idx].bot);
//         if (max_left>min_right || max_top > min_bot) {
//             continue;
//             //not overlapping
//         }else{
//             ratio1 = double((min_right - max_left) * (min_bot - max_top)) / double(Rects[sub_idx].width) / double (Rects[sub_idx].height);
//             ratio2 = double((min_right - max_left) * (min_bot - max_top)) / double(obj_bb_box[idx].right-obj_bb_box[idx].left) / double (obj_bb_box[idx].bot-obj_bb_box[idx].top);
//             cout << "obj #" << idx << "and stacked obj #" << sub_idx << "overlap ratio a:b:" << ratio1 << "\n";
//             cout << "obj #" << idx << "and stacked obj #" << sub_idx << "overlap ratio b:a:" << ratio2 << "\n";
//             if (ratio1>overlap_thresh && ratio2>overlap_thresh && strcmp(obj_bb_box[idx].type, Types[sub_idx])==0 ){
//                 overlap_flag = true;
//                 break;
//             }
//         }
//     }
//     return overlap_flag;    
// }
// Point2f get_centroid(struct obj_box box1){
//     Point2f centroid1((box1.left+box1.right) / 2,(box1.top+box1.bot) / 2);
//     return centroid1;
// }




//double get_centroid_distance(struct obj_box box1, struct obj_box box2){
//    int centroid1_x =(box1.left+box1.right) / 2;
//    int centroid1_y = (box1.top+box1.bot) / 2;
//    int centroid2_x = (box2.left+box2.right) / 2;
//    int centroid2_y = (box2.top+box2.bot) / 2;
//    // double centroid_dist = sqrt((centroid1_x-centroid2_x)*(centroid1_x-centroid2_x) + (centroid1_y-centroid2_y)*(centroid1_y-centroid2_y));
//    double centroid_dist = get_dist(centroid1_x,  centroid1_y,  centroid2_x,  centroid2_y);
//    // printf("getting centroid distance from box, %d %d %d %d,  and box,%d %d %d %d\n centroid distance: %f\n", box1.left,box1.top,box1.right,box1.bot,box2.left,box2.top,box2.right,box2.bot,centroid_dist);
//    return centroid_dist;
//}
//
//
//double get_centroid_direction(struct obj_box new_box, struct obj_box old_box){
//    int exist_centroid_x =(old_box.left+old_box.right) / 2;
//    int exist_centroid_y = (old_box.top+old_box.bot) / 2;
//    int new_centroid_x = (new_box.left+new_box.right) / 2;
//    int new_centroid_y = (new_box.top+new_box.bot) / 2;
//    // int new_centroid_x = (old_box.left+new_obj[new_idx].box.right)/2;
//    // int new_centroid_y = (new_obj[new_idx].box.top+new_obj[new_idx].box.bot)/2;
//    // int exist_centroid_x = (existing_obj[matched_idx].box.left+existing_obj[matched_idx].box.right)/2;
//    // int exist_centroid_y = (existing_obj[matched_idx].box.top+existing_obj[matched_idx].box.bot)/2;
//    return atan2(new_centroid_y-exist_centroid_y,new_centroid_x - exist_centroid_x) *180 / PI;
//    // return -pi to pi
//    //         90
//    // -/+180--------0
//    //        -90
//}
//
//double get_scale_distance(struct obj_box box1, struct obj_box box2){
//    double scale_dist = double((box1.right-box1.left)*(box1.top-box1.bot)) / double((box2.right-box2.left)*(box2.top-box2.bot));
//    return scale_dist;
//}
//bool same_box_test(double centroid_dist, double scale_dist, double centroid_thresh, double scale_thresh){
//    if (scale_dist > scale_thresh && scale_dist < 1/scale_thresh && (centroid_dist < centroid_thresh || centroid_thresh == -1 )) {
//        return true;
//    }else{
//        if (DEBUG) printf("centroid distance/thresh: %f/%f, scale gap/thresh: %f/%f\n", centroid_dist,centroid_thresh,scale_dist,scale_thresh);
//        return false;
//    }
//}
//
//// centroid thresh in pixels
//// scale thresh < 1
//bool same_box_test_from_box(struct obj_box box1, struct obj_box box2, double centroid_thresh, double scale_thresh){
//    double centroid_dist = get_centroid_distance(box1, box2);
//    double scale_dist = get_scale_distance(box1, box2);
//    return same_box_test(centroid_dist, scale_dist, centroid_thresh, scale_thresh);
//}

//Mat get_gray(Mat im0){
//    Mat curGray;
//    if (im0.channels() > 1) {
//        cvtColor(im0, curGray, CV_BGR2GRAY);
//    } else {
//        curGray = im0;
//    }
//    return curGray;
//}





//void printExistingObj(std::vector<Object> new_obj){
//    // cout << "Printing object vector:\n";
//    for (int new_idx = 0;(new_idx<int(new_obj.size())&&(new_obj[new_idx].box.left!=YOLO_MARK));new_idx++){
//        if (DEBUG) printf("Object # %d, [left,top,right,bot]:%d %d %d %d\n", new_obj[new_idx].id,new_obj[new_idx].box.left,new_obj[new_idx].box.top,new_obj[new_idx].box.right,new_obj[new_idx].box.bot);
//    }
//}




//Point2f get_box_avg_mov_vec(vector<Point2f> points[2], vector<uchar> status, obj_box box){
//    Rect rect = Rect(box.left,box.top,box.right-box.left,box.bot-box.top);
//    Rect rect_top_left = Rect(box.left,box.top,(box.right-box.left)/2,(box.bot-box.top)/2);
//    Rect rect_top_right = Rect(box.left+(box.right-box.left)/2,box.top,(box.right-box.left)/2,(box.bot-box.top)/2);
//    Rect rect_btm_left = Rect(box.left,box.top+(box.bot-box.top)/2,(box.right-box.left)/2,(box.bot-box.top)/2);
//    Rect rect_btm_right = Rect(box.left+(box.right-box.left)/2,box.top+(box.bot-box.top)/2,(box.right-box.left)/2,(box.bot-box.top)/2);
//    Point2f total_mv_vec;
//
//    total_mv_vec.x=0;
//    total_mv_vec.y=0;
//    double points_num = 0;
//
//    bool check_tl=false, check_tr=false, check_bl=false, check_br=false;
//    for (int i=0;i<points[1].size(); i++){
//        if (status[i] && points[1][i].inside(rect)){
//            points_num += 1;
//            // point moving vector
//            Point2f pt_mov_vec=points[1][i] - points[0][i];
//            // printf("points (%f,%f) moved to (%f,%f), mov_vec %f\n",points[0][i].x, points[0][i].y, points[1][i].x, points[1][i].y, norm(pt_mov_vec));
//            total_mv_vec = total_mv_vec + pt_mov_vec;
//            // check if the content in the box are all moving, method, check if there is optical flow in 4 sub region of the box
//            if (FLOW_CONSISTENCY_CHECK && norm(pt_mov_vec) > STOP_CENTROID_THRESH){
//                if (points[1][i].inside(rect_top_left)) check_tl=true;
//                if (points[1][i].inside(rect_top_right)) check_tr=true;
//                if (points[1][i].inside(rect_btm_left)) check_bl=true;
//                if (points[1][i].inside(rect_btm_right)) check_br=true;
//                // printf("points (%f,%f) in box [%d,%d,%d,%d]\n",points[1][i].x, points[1][i].y, box.left,box.right, box.top, box.bot);
//            }
//        }
//    }
//    Point2f avg_mv_vec(0,0);
//    if ( points_num != 0 && (!FLOW_CONSISTENCY_CHECK || ((check_tl&&check_br) && (check_tr && check_bl)) )){
//        avg_mv_vec.x = total_mv_vec.x / double(points_num);
//        avg_mv_vec.y = -total_mv_vec.y / double(points_num);// transform the coordinates, from left to right, top to bot, matrix coordinates is x(0->inf), y->(0->inf), real coordinate is x(-inf->inf), y(inf->-inf)
//        if (DEBUG) printf("New Box {top left][%d,%d] moved %f / %f = %f towards %f\n", box.left, box.top ,norm(total_mv_vec),points_num,norm(avg_mv_vec), atan2(avg_mv_vec.y, avg_mv_vec.x)*180/PI);
//    }
//    return avg_mv_vec;
//}

//
//vector<cv::Point2f> detect_keypoints(Mat im){
////    Ptr<FeatureDetector> detector =FeatureDetector::create(FEATURE_TYPE); //"GFTT", "BRISK"
//    vector<cv::Point2f> keypoints_ret;
//    cv::goodFeaturesToTrack(im, keypoints_ret, 500, 0.01, 10, Mat(), 3, 0, 0.04);
//    cornerSubPix(im, keypoints_ret, subPixWinSize, Size(-1,-1), termcrit);
////    int minHessian = 400;
////    Ptr<cv::BRISK> detector =BRISK::create(minHessian); //"GFTT", "BRISK"
////    // Ptr<FeatureDetector> detector =GFTTDetector::create(); //"GFFT", "BRISK"
////    detector->detect(im, keypoints_ret);
//    return keypoints_ret;
//}



// Mat describe_keypoints(vector<KeyPoint> keypoints, Mat im_gray){
//     // Ptr<DescriptorExtractor>    descriptor = DescriptorExtractor::create(FEATURE_TYPE);
//     Ptr<DescriptorExtractor>    descriptor = DescriptorExtractor::create(FEATURE_TYPE);
//     Mat described_features;
//     descriptor->compute(im_gray,keypoints,described_features);
//     return described_features;
// }






// Mat grabcut_total_with_yolobox(Mat im, Rect box){
//     ////////////////////////// grabcut //////////////////////
//     Mat mask, binMask, res;
//     mask.create( im.size(), CV_8UC1);
//     Mat bgdModel, fgdModel;
//     mask.setTo( GC_PR_BGD );
//     (mask(box)).setTo( Scalar(GC_PR_FGD) );
//     grabCut( im, mask, box, bgdModel, fgdModel, 3, GC_INIT_WITH_RECT );
//     getBinMask( mask, binMask );
//     im.copyTo( res, binMask );
//     return res;
// }



// ./MT_tracker ../../Egineering\ Project/test.mp4 ./data/VPC/23.txt 23 ./results/tmp/


void help(){
    cout <<"Input Argument: ./main PathToVideo PathToMetadata id ResultFolder\n"
         <<"An example input sequence:\n"
         <<"// ./main ./test.mp4 ./data/VPC/23.txt 23 ./results/tmp/\n"
         << endl
         <<"When running: \n"
         <<"press r to resume\n"
         <<"press p to pause\n"
         <<"press n to process next frame\n"
         <<"press q to quit\n";
}

void YOLOandAssociation(network& net,  char* CamID, Point2f frame_total_op_flow, ofstream& total_latency_file, ofstream& yolo_obj_file);
char Obj_Tracking(double scale, char* DatasetPath, Point2f frame_total_op_flow, Point2f total_op_flow, ofstream & latency_file, ofstream& opflow_file);



int main(int argc, char **argv){
    // if (MOBILE_CAM) {
    //     STOP_CENTROID_THRESH = STOP_CENTROID_THRESH / 3; //since it's 30 fps now
    //     MOVE_CENTROID_THRESH = MOVE_CENTROID_THRESH / 3; //since it's 30 fps now
    // }

    // initModules_nonfree();
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // CMT cmt;

    // vector<Mat> hist_vec;
    // int rect_idx = 0;   int disp_num = 50;
    int im_size = 448;
    //Initialization bounding box
    // Rect rect;    vector<Rect> Rects;    vector<char *> Types;
    //////////////////////////////////// Load YOLO////////////////////////////////////////
    // char *cfgfile = "../cfg/yolo-small.cfg";
    // char *weightfile = "../yolo-small.weights";


//    network net = parse_network_cfg(cfgfile);
//    load_weights(&net, weightfile);
//    set_batch_network(&net, 1);
//    srand(2222222);
    clock_t time;
    clock_t time_end;
    // draw bounding box and tag
    // int* bb_box;
    // struct obj_box* obj_bb_box_array;
    // vector<obj_box> obj_bb_box;
    // vector<obj_box> new_obj_bb_box;
    // Object moved_obj;

    // time measure opencv
    // cv::TickMeter tm;



    /////////////////////////////////////////// argument remains///////////////////////////////////////
    string input_path;
    string metadata_path;
    string CamID;
    string DatasetPath;
    char proto_path[100];
//    if (optind == argc - 4)
//    {
        input_path = argv[1];
        metadata_path = argv[1+1];
        CamID = argv[1+2];
        DatasetPath = argv[1+3];
        // SHOW_IMG =argv[optind+4];
        sprintf(proto_path,"./%s/../Cam%s.txt", DatasetPath.c_str(), CamID.c_str());
        cout << proto_path << endl;
//    }
//
//    else if (optind < argc - 4)
//    {
//        cerr << "Only 4 argument is allowed. input path, metadata paht, Camara ID, DatasetPath" << endl;
//        return 1;
//    }

//    char output_path[50];
//    if (TRACK_EVAL_OUTPUT){
//        sprintf(output_path,"./%s/cam%s_track_eval.txt",DatasetPath,id);
//    }else{
//        sprintf(output_path,"./%s/cam%s_obj_list.txt",DatasetPath,id);
//    }

    //Set up logging
//    FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
    FILELog::ReportingLevel() = logINFO;
    Output2FILE::Stream() = stdout; //Log to stdout

    //Create window
    // namedWindow(WIN_NAME);

//    VideoCapture cap;


    string cfgfile = "./YOLO/yolo_cfg/yolo.cfg";    string weightfile = "./yolo.weights";

    // the initialization order should be maintained....
    IO io((char*)DatasetPath.c_str(), (char*)CamID.c_str(),(char*)metadata_path.c_str());

    Cam camera(input_path, DatasetPath, CamID, metadata_path, cfgfile, weightfile, &io);


    Displayer displayer(&camera, camera.multiObjTracker);



//    //open metadata
//    ifstream metadata_file(metadata_path);
////    metadata_file.open(metadata_path);
//    if(!metadata_file.is_open()){
//        cerr<<"unable to open metadata."<< endl;
//        return -1;
//    }
//
//    ofstream obj_file(output_path);
//    if(!obj_file.is_open()){
//        cerr<<"unable to open output file " << output_path<< endl;
//        return -1;
//    }
//
//    // if (ORIENTATION_SANITY_CHECK){
//    ofstream opflow_file("./op_flow.txt");
//    if(!obj_file.is_open()){
//        cerr<<"unable to open output file op_flow.txt"<< endl;
//        return -1;
//    }
//    // }
//    char tmp_path[50];
//    sprintf(tmp_path,"./%s/cam%s_yolo_box.txt",DatasetPath,id);
//    ofstream yolo_obj_file(tmp_path);
//    // char tmp_path[50];
//    sprintf(tmp_path,"./%s/cam%s_latency.txt",DatasetPath,id);
//    ofstream latency_file(tmp_path);
//    sprintf(tmp_path,"./%s/cam%s_total_latency.txt",DatasetPath,id);
//    ofstream total_latency_file(tmp_path);

    // if (WRITE_VIDEO){
    //     char output[100]= "./data/out_video.mp4";
    //     int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC)); 
    //     Mat first_f,first_f_scaled;
    //     cap >> first_f;
    //     scale = double(first_f.rows) / double(RESIZEROWS);
    //     first_f_scaled = resize_Input_color(first_f,scale);
    //     // Size S = Size((int) first_f_scaled.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
    //                   // (int) first_f_scaled.get(CV_CAP_PROP_FRAME_HEIGHT));
    //     Size S = Size(first_f_scaled.cols,first_f_scaled.rows);
    //     // VideoWriter outputVideo;
    //     outputVideo.open( output, ex, cap.get(CV_CAP_PROP_FPS),S, true);
    //     if (!outputVideo.isOpened())
    //     {
    //         cout  << "Could not open the output video for write: " << output << endl;
    //         return -1;
    //     }
    // }
    // TODO: time alignment of metadata and video
//    frame_meta_time_alignment(cap,metadata_file);

    //begin the main loop

    // record for the objs

    //////////////////////////////////////////////////////Initialization: Get initial image for bounding box////////////////////////////////////////////////
    // time=clock();

//    metadata_file >> ts >> fps >> azimuth >> lat >> lon;






//    sprintf(rec_frame,"./data/recognized_frame_%s.jpg", id);
//    int RESIZEROWS=360;
//    double scale = 0;

    camera.loadFrame();
    if (DEBUG)cout << "scale =" << camera.getScale()<< "\n";

//    frame_id = 0;

    // meta_data=read_metadata(meta_data,metadata_file,frame_id);
    ///////////////////////////////////////////read metadata////////////////////////////


    // bool write_first_flag = true;

    // obj_bb_box = run_yolo(im0,net);
//    existing_obj = run_yolo(im0,camera.yolo.net,id);


/////////////////////// initialization for tracker ////////////////
    camera.initYOLOonInitialFrame();
    if (DEBUG){
        cout << "Print Initial YOLO result\n";
        io.printExistingObj();
    }
//    printExistingObj(camera.existing_obj);

    // for (int idx = 0;idx<int(existing_obj.size()) && (existing_obj[idx].box.left!=YOLO_MARK);idx++){

    //     Point2f vertices[4];
    //     cmt_vec[rect_idx].bb_rot.points(vertices);
    //     printf("CMT:: box %f,%f,%f,%f to be processed\n",vertices[0].x,vertices[2].y, vertices[2].x, vertices[0].y);
    // }











    /////////////////////////////////////////////////Main Loop ///////////////////////////////////////////



//    Point2f totalOpticalFlow;
//    Point2f frameTotalOpticalFlow;




    help();
    while(!end_flag && camera.loadFrame()){


//        cap >> im_0;


//
        if (SHOW_IMG){ displayer.showOriginalIm(WIN_NAME); }

        if (DECOUPLE_TO_FRAMES){
            char tmpstr[100];
            sprintf(tmpstr, "./%s/imgs/%d.jpg", DatasetPath.c_str(), camera.getCurFrameId());//camID(videopath)_timestamp_objID.jpg
            io.writeOriginalFrame(tmpstr);
        }

//        if (im_0.empty()) break; //Exit at end of video stream
//        frame_id++;
        if (camera.getCurFrameId() < START_FRAME){
            continue;
        }


        // if (frame_id<2000) continue;

        if (camera.getCurFrameId() % FRAME_PER_PROC != 0)continue;



//        curGray.copyTo(prevGray);


//        Obj_Tracking(camera.scale, DatasetPath, frameTotalOpticalFlow, totalOpticalFlow, latency_file, opflow_file);
//        char MultiObjTracker::TrackMultiObj(cv::Mat prevGray, cv::Mat curGray, std::vector<Object> & existing_obj, std::vector<Object> & moved_obj ){// cv::Mat should be automatically using reference, watch out...
        camera.multiObjTracker->TrackMultiObj();
        if (SHOW_KYPTS){ displayer.drawKPTMatchesOnPrevIm();}
        char key  = displayer.displayExistingObjOnPrevIm(0, 255, 0);

        end_flag = camera.process_key(key);

        // debug
        // waitKey(100000);

        ////////////////////////////////////////// YOLO + bbox association /////////////////////////////////


        if (camera.processTrigger()){

            int assoc_start= cvGetTickCount();

            camera.runYOLOonNewFrame();
            camera.MultiObjMatching();

            int total_end = cvGetTickCount();

            if (TRACK_EVAL_OUTPUT){
                // double total_processtime = (double)(total_end-assoc_start + overhead_end -overhead_start)/ (cvGetTickFrequency()) / 1000000;
                double total_processtime = (double)(total_end-assoc_start)/ (cvGetTickFrequency()) / 1000000;
                io.total_latency_file << total_processtime << endl;
            }
        }else{
            camera.deadReckoning();
        }

//        YOLOandAssociation(camera.yolo.net,  id, frameTotalOpticalFlow, total_latency_file, yolo_obj_file);



        if (DEBUG) cout << "after yolo association\n";
        // printExistingObj(new_obj);
        ///////////////////////////////// logging and output //////////////////////////
        if (DEBUG) cout << "Logging ...frame " << camera.getCurFrameId() << endl;
        //////////////////////////////// logging using protobuf & write each frame to file to see the bounding box associated correctly //////////////////
        io.logObjects();

    }
    cout << "Finished processing in total " << camera.getCurFrameId() << "frames\n";

    // fstream final_output("./Cam1.txt", ios::out | ios::trunc | ios::binary);
    io.writeProtobuf(proto_path);
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}


//
//
//void YOLOandAssociation(network& net,  char* id, Point2f frameTotalOpticalFlow, ofstream& total_latency_file, ofstream& yolo_obj_file){
//    // int obj_idx = 0;
//    // int idx = 0;
//    // run yolo
//    // new_obj_bb_box = run_yolo(im,net);
//    // new_obj.clear();
//    // moved_obj.clear();
//    vector<Object> new_obj;
//    im.copyTo(prevIm);
//    // int id = 0;
//    // while (true){
//    //     cout << id++ << endl;
//    int assoc_start;
//    if (camera->getCurFrameId % YOLO_FREQ != 0){
//        // USE TRACKING ONLY, NO YOLO NEW BOX ASSOCIATION
//        camera.existing_obj = moved_obj;
//        assoc_start = cvGetTickCount();
//    }else{
//
//        cout << "YOLO ... frame " << camera->getCurFrameId << endl;
//        new_obj = run_yolo(im0,net,id);
//        if (DEBUG) cout << "Yolo result:\n";
//        print_obj(new_obj);
//
//        // }
//        // cout << "test\n";
//        // bounding box association
//        assoc_start = cvGetTickCount();
//        // phase 1: compare new box with existing box, filter out not moving
//        for (int new_idx = 0;(new_idx<int(new_obj.size())&&(new_obj[new_idx].box.left!=YOLO_MARK));new_idx++){
//
//            if (YOLO_DETECTION_OUTPUT){
//                yolo_obj_file << camera->getCurFrameId << ", "<< "-1" << ", " << new_obj[new_idx].box.left << ", "<< new_obj[new_idx].box.top << ", "
//                              << new_obj[new_idx].box.right - new_obj[new_idx].box.left << ", " << new_obj[new_idx].box.bot - new_obj[new_idx].box.top
//                              << ", -1, -1, -1, -1" << endl;
//            }
//
//            // phase 1:
//            double min_centroid_dist = 1000;
//            int matched_idx = -1;
//
//            for (int exist_idx = 0;exist_idx<int(camera.existing_obj.size())&&(camera.existing_obj[exist_idx].box.left!=YOLO_MARK);exist_idx++){
//                // shouldn't check alive, because you need dead box to rule out new box as well
//
//                double tmp_dist = get_centroid_distance(camera.existing_obj[exist_idx].box,new_obj[new_idx].box);
//                // printf("getting centroid distance from box, %d %d %d %d,  and box,%d %d %d %d\n centroid distance: %f\n", existing_obj[exist_idx].box.left,existing_obj[exist_idx].box.top,existing_obj[exist_idx].box.right,existing_obj[exist_idx].box.bot,new_obj[new_idx].box.left,new_obj[new_idx].box.top,new_obj[new_idx].box.right,new_obj[new_idx].box.bot,tmp_dist);
//                double tmp_scale = get_scale_distance(camera.existing_obj[exist_idx].box,new_obj[new_idx].box);
//                if ( tmp_dist < min_centroid_dist && same_box_test(tmp_dist,tmp_scale,STOP_CENTROID_THRESH, STOP_SCALE_THRESH)){
//                    min_centroid_dist = tmp_dist;
//                    matched_idx = exist_idx;
//                }
//
//            }
//            if (matched_idx!=-1){
//                // mark don't care, not moving
//                if (DEBUG) printf("Object %d is seen not moving\n", camera.existing_obj[matched_idx].id);
//                camera.existing_obj[matched_idx].alive = false;
//                camera.existing_obj[matched_idx].evict = false;
//                camera.existing_obj[matched_idx].evict_count = 0;
//                // mark them matched by giving -1 id
//                new_obj[new_idx].id = -1;
//                // NO YOU CAN NOT delete cmt vector, CMT VECTOR INDEX MUST MATCH OBJ_BB_BOX INDEX
//                continue;
//            }
//            // printf("Object %d might moved\n", existing_obj[matched_idx].id);
//        }
//        // phase 2: compare new box with cmt prediction, match moved box, create new box and cmt instance if no match
//        for (int new_idx = 0;new_idx<int(new_obj.size())&&(new_obj[new_idx].box.left!=YOLO_MARK);new_idx++){
//
//            Point2f new_box_mov_vec = get_box_avg_mov_vec(points, status, new_obj[new_idx].box);
//
//
//
//            if (new_obj[new_idx].id!=-1 && check_movement(new_box_mov_vec,frameTotalOpticalFlow)){
//                // box is moving
//                double min_centroid_dist = 10000;
//                int matched_idx = -1;
//                // Point2f min_centroid;
//                int new_centroid_x = (new_obj[new_idx].box.left+new_obj[new_idx].box.right)/2;
//                int new_centroid_y = (new_obj[new_idx].box.top+new_obj[new_idx].box.bot)/2;
//                for (int moved_idx = 0;moved_idx<int(moved_obj.size())&&(moved_obj[moved_idx].box.left!=YOLO_MARK);moved_idx++){
//                    // should not check obj_bb_box if moving first, here is the only chance for a stopped car to be recognized again.
//                    // double tmp_dist = norm(new_centroid-exist_centroid);
//                    double tmp_dist = get_centroid_distance(moved_obj[moved_idx].box,new_obj[new_idx].box);
//                    double tmp_scale = get_scale_distance(moved_obj[moved_idx].box,new_obj[new_idx].box);
//                    if ( tmp_dist < min_centroid_dist && same_box_test(tmp_dist,tmp_scale,MOVE_CENTROID_THRESH, MOVE_SCALE_THRESH)){
//                        if (USE_LKT){
//                            double total = 0;
//                            double match = 0;
//                            for ( int i=0;i<lkt_obj[moved_idx].idx_vec.size();i++){
//                                if (status[lkt_obj[moved_idx].idx_vec[i]]){
//                                    total = total+1.0;
//                                    Rect rect_tmp = Rect(new_obj[new_idx].box.left,new_obj[new_idx].box.top,new_obj[new_idx].box.right-new_obj[new_idx].box.left,new_obj[new_idx].box.bot-new_obj[new_idx].box.top);
//                                    if (points[1][lkt_obj[moved_idx].idx_vec[i]].inside(rect_tmp) ){
//                                        match = match+1.0;
//                                    }
//                                }
//                            }
//                            if ( (total >=3 && match / total > KEYPOINT_RATIO_THRESH) || total <3){
//                                min_centroid_dist = tmp_dist;
//                                // min_centroid = exist_centroid;
//                                matched_idx = moved_idx;
//                            }else{
//                                if (DEBUG) printf("Yolo box #%d, keypoints not matching existing Obj %d, total: %f, match: %f\n", new_idx, camera.existing_obj[moved_idx].id,  total,match);
//                            }
//                        }else{
//                            min_centroid_dist = tmp_dist;
//                            // min_centroid = exist_centroid;
//                            matched_idx = moved_idx;
//                        }
//                    }else{
//                        if (DEBUG) printf("Yolo box #%d, box distance not matching to existing Obj %d, dist: %f, scale_dist %f \n", new_idx, camera.existing_obj[moved_idx].id, tmp_dist, tmp_scale);
//                    }
//                }
//                if (matched_idx!=-1){
//                    new_obj[new_idx].id = camera.existing_obj[matched_idx].id;
//                    // mark moved_obj_box matched by giving the object id that it has. by marking the tracked flag true now
//                    // moved_obj[matched_idx].id = existing_obj[matched_idx].id;
//                    moved_obj[matched_idx].tracked = true;
//
//                    // update moving direction
//                    int exist_centroid_x = (camera.existing_obj[matched_idx].box.left+camera.existing_obj[matched_idx].box.right)/2;
//                    int exist_centroid_y = (camera.existing_obj[matched_idx].box.top+camera.existing_obj[matched_idx].box.bot)/2;
//                    // existing_obj[matched_idx].displacement = get_dist(new_centroid_x,new_centroid_y,exist_centroid_x,exist_centroid_y);
//                    // existing_obj[matched_idx].direction = atan2(new_centroid_y-exist_centroid_y,new_centroid_x - exist_centroid_x) *180 / PI;
//                    if (MOBILE_CAM){
//                        camera.existing_obj[matched_idx].displacement = norm(new_box_mov_vec - frameTotalOpticalFlow);
//                        camera.existing_obj[matched_idx].direction = atan2((new_box_mov_vec - frameTotalOpticalFlow).y, (new_box_mov_vec - frameTotalOpticalFlow).x) *180 / PI;
//                    }else{
//                        camera.existing_obj[matched_idx].displacement = norm(new_box_mov_vec);
//                        camera.existing_obj[matched_idx].direction = atan2(new_box_mov_vec.y, new_box_mov_vec.x) *180 / PI;
//                    }
//
//
//                    if (DEBUG) printf("moving from (%d,%d) to (%d,%d), dist: %f, dir: %f\n", exist_centroid_x, exist_centroid_y, new_centroid_x, new_centroid_y,  camera.existing_obj[matched_idx].displacement,camera.existing_obj[matched_idx].direction );
//                    // return -pi to pi
//                    //         90
//                    // -/+180--------0
//                    //        -90
//
//                    // if (new_centroid_x < exist_centroid_x){
//                    //     existing_obj[matched_idx].direction+=180;
//                    // }
//                    // existing_obj[matched_idx].direction+=90;
//                    // existing_obj[matched_idx].displacement = norm(new_centroid-exist_centroid);
//                    // update TO BE A moving object
//                    if (camera.existing_obj[matched_idx].displacement<STOP_CENTROID_THRESH){ // TODO, check if need to replace it with check_movement function here as well
//                        camera.existing_obj[matched_idx].alive = false;
//                        camera.existing_obj[matched_idx].evict = false;
//                        camera.existing_obj[matched_idx].evict_count = 0;
//                        continue;
//                    }else{
//                        if (DEBUG) printf("Object %d moved %f towards %f \n", camera.existing_obj[matched_idx].id,  camera.existing_obj[matched_idx].displacement,camera.existing_obj[matched_idx].direction);
//                        // printf("    Old centroid: %d,%d   New centroid: %d,%d\n", exist_centroid_x,exist_centroid_y, new_centroid_x, new_centroid_y);
//                        camera.existing_obj[matched_idx].alive = true;
//                        camera.existing_obj[matched_idx].evict = false;
//                        camera.existing_obj[matched_idx].evict_count = 0;
//                        camera.existing_obj[matched_idx].move_count++;
//                        if (!check_boundary(im.cols, im.rows, camera.existing_obj[matched_idx].box)){
//                            // too close to the boundary : either enter or exit
//                            if ( camera.existing_obj[matched_idx].status.compare(ENTER) ==0 ){
//                                // keep status of enter
//                            }else if (camera.existing_obj[matched_idx].status.compare(MOVE) ==0 ){
//                                camera.existing_obj[matched_idx].status = EXIT;
//                            }
//                        }else{
//                            // not close to boundary, this could be marked as move
//                            camera.existing_obj[matched_idx].status = MOVE;
//                        }
//                    }
//                    // TODO: judge the status in the cloud pls, combining the field view direction
//                    // update status: front mid rear, probably wouldn't matter at all
//                    // if (existing_obj[matched_idx].direction < 45 || ){
//
//                    // }
//
//                    // update existing bbox to be the new one
//                    camera.existing_obj[matched_idx].box = new_obj[new_idx].box;
//                    // refresh cmt last image and bounding box as welll
//                    if (USE_CMT){
//                        Rect rect_tmp = Rect(camera.existing_obj[matched_idx].box.left,camera.existing_obj[matched_idx].box.top,camera.existing_obj[matched_idx].box.right-camera.existing_obj[matched_idx].box.left,camera.existing_obj[matched_idx].box.bot-camera.existing_obj[matched_idx].box.top);
//                        //legalize the bbox
//                        rect_tmp = legalize_box(im0,rect_tmp);
//
//                        CMT cmt_tmp;
//                        cmt_tmp.initialize(im0_gray, rect_tmp, im0);
//                        cmt_vec.erase(cmt_vec.begin()+matched_idx);
//                        cmt_vec.insert(cmt_vec.begin()+matched_idx,cmt_tmp);
//                    }
//
//                    // TODO: add cld? the closest match is not robust when there are two cars moving towards each other.
//                }else{
//                    // new instance
//                    // cout << "Object No. "  << "\n";
//                    Object_ID++;
//                    if (DEBUG) printf("New Object %d \n",Object_ID);
//                    // obj_box tmp_box = new_obj[new_idx].box;
//                    // obj_bb_box.push_back(tmp_box);
//                    Object tmp_obj;
//                    tmp_obj.box = new_obj[new_idx].box;
//                    tmp_obj.last_box = new_obj[new_idx].box;
//                    tmp_obj.id = Object_ID;
//                    tmp_obj.alive = true;
//                    tmp_obj.tracked = true;
//                    tmp_obj.evict = false;
//                    tmp_obj.status = ENTER;
//                    tmp_obj.displacement = 0;
//                    tmp_obj.direction = 0;
//                    tmp_obj.evict_count = 0;
//                    tmp_obj.move_count = 0;
//                    camera.existing_obj.push_back(tmp_obj);
//                    new_obj[new_idx].id = Object_ID;
//
//                    if (USE_CMT){
//                        //creating the rectangle for bb_box
//                        Rect rect_tmp = Rect(new_obj[new_idx].box.left,new_obj[new_idx].box.top,new_obj[new_idx].box.right-new_obj[new_idx].box.left,new_obj[new_idx].box.bot-new_obj[new_idx].box.top);
//                        //legalize the bbox
//                        rect_tmp = legalize_box(im0,rect_tmp);
//                        CMT cmt_tmp;
//                        cmt_tmp.initialize(im0_gray, rect_tmp, im0);
//                        cmt_vec.push_back(cmt_tmp);
////                            FILE_LOG(logINFO) << "Using " << rect_tmp.x << "," << rect_tmp.y << "," << rect_tmp.x + rect_tmp.width << "," << rect_tmp.y+rect_tmp.height
////                                              << "for objs #" << int(existing_obj.size()) << " as initial bounding box.";
//
//                    }
//
//                }
//            }
//        }
//        // phase 3: go through cmt prediction, remove box no longer valid, no match
//        for (int moved_idx = 0;moved_idx<int(moved_obj.size())&&(moved_obj[moved_idx].box.left!=YOLO_MARK);moved_idx++){
//            //TODO: remove cmt vec not matched and obj_bb_box.
//            // if (moved_obj[moved_idx].id==-1){
//            if (moved_obj[moved_idx].tracked==false){
//                if (camera.existing_obj[moved_idx].alive){
//                    if (DEBUG) printf("Object %d: Lost track.\n",camera.existing_obj[moved_idx].id);
//
//                }
//                camera.existing_obj[moved_idx].alive=false;
//                camera.existing_obj[moved_idx].evict=true;
//
//                // should update the box position according to keypoints in case in future recognized again
//                Point2f exist_box_mov_vec = get_box_avg_mov_vec(points, status, camera.existing_obj[moved_idx].box);
//                exist_box_mov_vec.y = -exist_box_mov_vec.y;
//                Point2f left_top(camera.existing_obj[moved_idx].box.left,camera.existing_obj[moved_idx].box.top);
//                Point2f right_bot(camera.existing_obj[moved_idx].box.right,camera.existing_obj[moved_idx].box.bot);
//                left_top += exist_box_mov_vec;
//                right_bot += exist_box_mov_vec;
//                camera.existing_obj[moved_idx].box.left = left_top.x < 0 ? 0:left_top.x;
//                camera.existing_obj[moved_idx].box.top = left_top.y < 0 ? 0:left_top.y;
//                camera.existing_obj[moved_idx].box.right = right_bot.x > im.cols? im.cols: right_bot.x ;
//                camera.existing_obj[moved_idx].box.bot = right_bot.y > im.rows? im.rows:right_bot.y ;
//
//                // invalid anymore,
//                // shouldn't erase, erase will be recognized again
//                // existing_obj.erase(existing_obj.begin()+moved_idx);
//                // cmt_vec.erase(cmt_vec.begin()+moved_idx);
//            }
//        }
//
//        // mark for eviction
//        for (int exist_idx = 0;exist_idx<int(camera.existing_obj.size())&&(camera.existing_obj[exist_idx].box.left!=YOLO_MARK);exist_idx++){
//            if (camera.existing_obj[exist_idx].alive == false && camera.existing_obj[exist_idx].evict){
//                if (camera.existing_obj[exist_idx].evict_count++ / fps >= EVICT_TRHESH){
//                    // object eviction for not seen from yolo for a long time.
//                    if (DEBUG) printf("Erasing Object %d\n", camera.existing_obj[exist_idx].id);
//                    camera.existing_obj.erase(camera.existing_obj.begin()+exist_idx);
//                    if (USE_CMT){
//                        cmt_vec.erase(cmt_vec.begin()+exist_idx);
//                    }
//                    break; // evict one at a time, otherwise idx will not match
//                }
//            }else{
//                camera.existing_obj[exist_idx].evict_count = 0;
//            }
//        }
//    }
//
//    int total_end = cvGetTickCount();
//
//    if (TRACK_EVAL_OUTPUT){
//        // double total_processtime = (double)(total_end-assoc_start + overhead_end -overhead_start)/ (cvGetTickFrequency()) / 1000000;
//        double total_processtime = (double)(total_end-assoc_start)/ (cvGetTickFrequency()) / 1000000;
//        total_latency_file << total_processtime << endl;
//    }
//}
//
//
//char Obj_Tracking(double scale, char* DatasetPath, Point2f frameTotalOpticalFlow, Point2f totalOpticalFlow, ofstream & latency_file, ofstream& opflow_file){
//    ///////////////////// extract keypoints from previous frame /////////////////
//    // if (USE_LKT){
//    // goodFeaturesToTrack(prevGray, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
//    // cornerSubPix(prevGray, points[0], subPixWinSize, Size(-1,-1), termcrit);
//    // need gfft for movement detection anyway
//
//    int start = cvGetTickCount();
//    points[0] = detect_keypoints(prevGray);
////        vector<cv::Point2f> kypts = detect_keypoints(prevGray);
//    int mid = cvGetTickCount();
////        points[0] = convert_keypoints_to_point(kypts);
//    double detect_time = (double)(mid-start)/ (cvGetTickFrequency()) / 1000000;
//    double describe_time= 0;
//    Mat described_features;
//    int end;
//    size_t sizeInBytes;
//    // feature discriptor no longer supported by opencv 3.0
//    // Seems to have integrated into the detector of function detectionandcompute
//    // if (FEATURE_TYPE=="ORB" || FEATURE_TYPE=="BRISK" ){
//    //     described_features= describe_keypoints(kypts,prevGray);
//    //     end = cvGetTickCount();
//    //     sizeInBytes= described_features.total() * described_features.elemSize();
//    //     describe_time = (double)(end-mid)/ (cvGetTickFrequency()) / 1000000;
//    // }
//    // // }
//    //store keypoints for each box
//    lkt_obj.clear();
//    for (int idx = 0;idx<int(camera.existing_obj.size()) && (camera.existing_obj[idx].box.left!=YOLO_MARK);idx++){
//        LKT_IDX tmp_lkt_vec;
//        Rect rect_tmp = Rect(camera.existing_obj[idx].box.left,camera.existing_obj[idx].box.top,camera.existing_obj[idx].box.right-camera.existing_obj[idx].box.left,camera.existing_obj[idx].box.bot-camera.existing_obj[idx].box.top);
//        // vector<KeyPoint> tmp_kypts;
//        for( int i=0; i < points[0].size(); i++ ){
//            if (points[0][i].inside(rect_tmp)){
//                tmp_lkt_vec.idx_vec.push_back(i);
//                // tmp_kypts.push_back(kypts[i]);
//            }
//        }
//        lkt_obj.push_back(tmp_lkt_vec);
//        // Mat tmp_desc = describe_keypoints(tmp_kypts,prevGray);
//        // size_t sizePerObj = tmp_desc.total() * tmp_desc.elemSize();
//        // if (TRACK_EVAL_OUTPUT){
//        //     latency_file << sizePerObj << " ";
//        // }
//    }
//
//    if (SHOW_IMG){imshow(WIN_NAME, im_0);}
////    if (im_0.empty()) break; //TODO: Exit at end of video stream
//    // read metadata
//    // meta_data=read_metadata(meta_data,metadata_file,frame_id);
//    // convert to gray
//    im = resize_Input_color(im_0,scale);
//    im_gray = get_gray(im);
//    // if (im.channels() > 1) {
//    //     cvtColor(im, im_gray, CV_BGR2GRAY);
//    // } else {
//    //     im_gray = im;
//    // }
//    // store a no box version for thumbnail and color layout
//
//    im.copyTo(im0);
//    im_gray.copyTo(im0_gray);
//    ///////////////////print existing vector /////////////
//    // printf("print existing obj:\n");
//    // printExistingObj(existing_obj);
//
//    if (DECOUPLE_TO_FRAMES){
//        char tmpstr[100];
//        sprintf(tmpstr, "./%s/imgs/%d.jpg", DatasetPath, camera->getCurFrameId);//camID(videopath)_timestamp_objID.jpg
//        // cout << tmpstr << endl;
//        imwrite(tmpstr,im0);
//    }
//
//    int overhead_start = cvGetTickCount();
//    calcOpticalFlowPyrLK(prevGray, im0_gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
//    int overhead_end = cvGetTickCount();
//    ///////////////////////////////////// CMT processing //////////////////////////
//    /////////////////////////////////////  CMT predicting object movement, Tracking the object, update exisiting objects to moved objects //////////////////////////////////////////////////////////////////
//
//    char key;
//    if (USE_CMT){
//        if (DEBUG) printf("total %d obj, %d cmt\n", int(camera.existing_obj.size()), int(cmt_vec.size()));
//        cout << "CMT ... frame " << camera->getCurFrameId << endl;
//        for (int rect_idx=0;rect_idx<int(cmt_vec.size());rect_idx++){
//            Point2f vertices[4];
//            cmt_vec[rect_idx].bb_rot.points(vertices);
//            // printf("CMT:: box %f,%f,%f,%f to be processed\n",vertices[0].x,vertices[2].y, vertices[2].x, vertices[0].y);
//
//
//            // time=clock();
//            // tm.start();
//            int start = cvGetTickCount();
//            cmt_vec[rect_idx].processFrame(im_gray);
//            // time_end = clock();
//            int end = cvGetTickCount();
//            // tm.stop();
//            cmt_vec[rect_idx].bb_rot.points(vertices);
//
//            // printf("CMT::processFrame of size (%d,%d) in %f seconds.\n", im_gray.rows, im_gray.cols, sec(time_end-time));
//            double processtime = (double)(end-start)/ (cvGetTickFrequency()) / 1000000;
//            if (DEBUG) printf("CMT::processFrame of size (%d,%d) in %f seconds.\n", im_gray.rows, im_gray.cols, processtime);
//            if (DEBUG) printf("CMT:: box %f,%f,%f,%f processed\n",vertices[0].x,vertices[2].y, vertices[2].x, vertices[0].y);
//            if (TRACK_EVAL_OUTPUT){
//                latency_file << processtime << endl;
//            }
//            // FILE_LOG(logINFO) << "#" << frame_id << ", OBJ: "<< existing_obj[rect_idx].id<<", active: " << cmt_vec[rect_idx].points_active.size();
//        }
//        // vector<obj_box> moved_obj_box;
//        // cout << "im size" << im.rows << " x " << im.cols << "\n";
//        // cout << "Displaying ... frame " << frame_id << endl;
//        key = display_multi(im, cmt_vec, camera.existing_obj);
////        if(key == 'q') {
////            end_flag = true;
////            break;
////        }
//
//
//        if (write_first_flag){
//            imwrite(rec_frame,im);
//            write_first_flag=false;
//        }
//        moved_obj = get_obj_box_from_cmt_vec(cmt_vec,camera.existing_obj);
//    }else if (USE_LKT){
//        status.clear();
//        err.clear();
//
//        // track keypoints
//        //Calculate forward optical flow for prev_location
//        // time=clock();
//        int start = cvGetTickCount();
//
//
//        calcOpticalFlowPyrLK(prevGray, im0_gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
//        frameTotalOpticalFlow.x=0;frameTotalOpticalFlow.y=0;
//        int frame_keypt_num=0;
//        for (int i = 0; i<points[0].size(); i++){
//
//            if (status[i]){
//                frameTotalOpticalFlow = frameTotalOpticalFlow + points[1][i]-points[0][i];
//                frame_keypt_num++;
//                totalOpticalFlow = totalOpticalFlow + points[1][i]-points[0][i];
//                keypt_num++;
//            }
//        }
//        frameTotalOpticalFlow.x = frameTotalOpticalFlow.x / frame_keypt_num;
//        frameTotalOpticalFlow.y = -frameTotalOpticalFlow.y / frame_keypt_num;
//        if (camera->getCurFrameId % (fps/10) == 0){
//            totalOpticalFlow.x = totalOpticalFlow.x / keypt_num;
//            totalOpticalFlow.y = -totalOpticalFlow.y / keypt_num;
//            if (ORIENTATION_SANITY_CHECK){
//                opflow_file << "frame ID: " << camera->getCurFrameId << ", x:"<< totalOpticalFlow.x << ", y:" << totalOpticalFlow.y << ", total flow: " << norm(totalOpticalFlow) << ", direction: " << atan2(totalOpticalFlow.y, totalOpticalFlow.x) << endl;
//            }
//            totalOpticalFlow.x=0;totalOpticalFlow.y=0;
//            keypt_num = 0;
//        }
//
//        vector<Point2f> points_back;
//        vector<unsigned char> status_back;
//        vector<float> err_back; //Needs to be float
//
//        //Calculate backward optical flow for prev_location
//        calcOpticalFlowPyrLK(im0_gray, prevGray, points[1], points_back, status_back, err_back);
//
//        //Traverse vector backward so we can remove points on the fly
//        for (int i = points[0].size()-1; i >= 0; i--)
//        {
//            float l2norm = norm(points_back[i] - points[0][i]);
//
//            bool fb_err_is_large = l2norm > 30; // 30 threshold adopt from open CMT
//
//            if (fb_err_is_large || !status[i] || !status_back[i])
//            {
//                // points_tracked.erase(points_tracked.begin() + i);
//
//                //Make sure the status flag is set to 0
//                status[i] = 0;
//            }
//
//        }
//
//        // print circles~~ debug
//        if (SHOW_KYPTS){
//            for( int i = 0; i < points[1].size(); i++ )
//            {
//                if( status[i] ){
//                    circle( im, points[0][i], 3, Scalar(255,0,0), -1, 8);
//                    circle( im, points[1][i], 3, Scalar(0,255,0), -1, 8);
//                    line( im, points[0][i], points[1][i], Scalar(0,0,255));
//
//                }
//            }
//
//        }
//
//        // prepare moved obj
//        moved_obj = camera.existing_obj;
//        for (int moved_idx=0;moved_idx<int(moved_obj.size());moved_idx++){
//            // moved_obj[moved_idx].id = -1;
//            moved_obj[moved_idx].tracked = false;
//            Point2f moved_box_mov_vec = get_box_avg_mov_vec(points, status, moved_obj[moved_idx].box);
//            if (DEBUG) printf("last moved object: %d,%d,%d,%d, of img %d,%d\n", moved_obj[moved_idx].box.left,moved_obj[moved_idx].box.right, moved_obj[moved_idx].box.top,moved_obj[moved_idx].box.bot, im.rows, im.cols);
//            moved_obj[moved_idx].box.left += moved_box_mov_vec.x;
//            moved_obj[moved_idx].box.top += -moved_box_mov_vec.y;
//            moved_obj[moved_idx].box.right += moved_box_mov_vec.x;
//            moved_obj[moved_idx].box.bot += -moved_box_mov_vec.y;
//            if (DEBUG) printf("moved object: %d,%d,%d,%d, of img %d,%d\n", moved_obj[moved_idx].box.left,moved_obj[moved_idx].box.right, moved_obj[moved_idx].box.top,moved_obj[moved_idx].box.bot, im.rows, im.cols);
//        }
//        // time_end = clock();
//        int end = cvGetTickCount();
//        double processtime = (double)(end-start)/ (cvGetTickFrequency()) / 1000000;
//        if (DEBUG) printf("KLT::processFrame of size (%d,%d) in %f seconds.\n", im_gray.rows, im_gray.cols, processtime);
//        if (TRACK_EVAL_OUTPUT){
//            latency_file << detect_time << "+" << describe_time << "+" << processtime << "=" << detect_time+describe_time +processtime << ", size:" << sizeInBytes<< endl;
//        }
//        // printf("KLT::processFrame of size (%d,%d) in %f seconds.\n", im_gray.rows, im_gray.cols, sec(time_end-time));
//        // show existing obj
//        if (DEBUG) cout << "im size" << im.rows << " x " << im.cols << "\n";
//        key = display_existing_obj(prevIm, camera.existing_obj,0,255,0);
//    }else{
//        moved_obj = camera.existing_obj;
//        for (int moved_idx=0;moved_idx<int(moved_obj.size());moved_idx++){
//            // moved_obj[moved_idx].id = -1;
//        }
//        // show existing obj
//        key = display_existing_obj(im, camera.existing_obj,0,255,0);
//    }
//    return key;
//}