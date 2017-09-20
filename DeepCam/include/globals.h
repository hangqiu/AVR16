//
// Created by nsl on 9/16/16.
// Gloabal variables and functions
//

#ifndef MOBILEVIDEONET_CLEAN_GLOBALS_H
#define MOBILEVIDEONET_CLEAN_GLOBALS_H

#include <iostream>
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <obj_record.pb.h>


#ifdef __cplusplus
extern "C" {
#endif

#include "DeepCam/YOLO/detection.h"

#ifdef __cplusplus
}
#endif

#define PI 3.14159265

using namespace std;
using namespace cv;

extern bool PAUSE_FLAG;
extern bool FLOW_CONSISTENCY_CHECK;
extern bool DEBUG;
extern const bool SHOW_IMG;
extern const bool USE_LKT;


extern double STOP_CENTROID_THRESH;

extern double STOP_SCALE_THRESH;
extern bool ORIENTATION_SANITY_CHECK;
// bool ORIENTATION_SANITY_CHECK = true;
extern bool TRACK_EVAL_OUTPUT;
extern bool YOLO_DETECTION_OUTPUT; // detection output frequency is the same with yolo_freq
extern bool GRABCUT_FLAG;
extern bool DECOUPLE_TO_FRAMES;
extern bool MOBILE_CAM;
extern bool BOX_AREA_FILTER;
extern bool SHOW_KYPTS_THUMBNAIL;
extern bool SHOW_KYPTS;

extern const int YOLO_MARK;
extern double MOVE_CENTROID_THRESH; // CAN MOVE ANYWHERE FUNCTION WILL FIND CLOSEST
extern const double MOVE_SCALE_THRESH; // CAN CHANGE QUITE A BIT
extern const double KEYPOINT_RATIO_THRESH;

extern const int BOUNDARY_THRESH; //pixel
extern const double BOX_AREA_THRESH; // boxes less than box area will not be considered...

extern const double DIRECTION_THRESH;
// obj flag
// const string STOP="Stop";
// const string MOVE="Move";
// obj status
extern const string ENTER;
extern const string MOVE;
extern const string EXIT;
extern const string UNKNOWN;
// obj view
extern const string FRONT;
extern const string MID;


extern const string REAR;
extern const double EVICT_TRHESH; // seconds
extern const double OUTPUT_FREQ ; // seconds output cld and object per second for moving object

extern const int FRAME_PER_PROC;// process per FRAME_PER_PROC number of frames
extern int YOLO_FREQ;

extern int START_FRAME;

extern const string WIN_NAME;

extern const int RESIZEROWS;

// double get_centroid_distance(struct obj_box box1, struct obj_box box2){
extern double get_dist(int centroid1_x, int centroid1_y, int centroid2_x, int centroid2_y);



//Mat load_frame_dump_metadata(VideoCapture cap,ifstream& metadata_file){
//    Mat frame;
//    cap >> frame;
//    if (frame.empty()) return frame;
//    // meta_data=read_metadata(meta_data,metadata_file,frame_id);
////    FrameId++;
//    return frame;
//}
//
//double get_frame_for_scale(int RESIZEROWS, VideoCapture cap, ifstream & metadata_file){
//
//    Mat frame = load_frame_dump_metadata(cap,metadata_file);
//    return double(frame.rows) / double(RESIZEROWS);
//}
//
//void frame_meta_time_alignment(VideoCapture cap, ifstream & metadata_file){
//    int video_timestamp = cap.get( CV_CAP_PROP_POS_MSEC );
//    Mat tmp;
//    int gap=0;
//    meta_data=pop_metadata(meta_data,metadata_file);
//    // cout << "video ts: " << video_timestamp << " meta ts: "<<meta_data.timestamp<< endl;
//    //TODO: video timstamp is currently returned 0. what's happening?
//    if (meta_data.timestamp<video_timestamp){
//        while (meta_data.timestamp<video_timestamp){
//            meta_data=pop_metadata(meta_data,metadata_file);
//        }
//        cout << "After alignment,  meta ts: "<<meta_data.timestamp<< endl;
//    }else{
//        gap = meta_data.timestamp-video_timestamp;
//        for (int i=0;i<gap / 33 ;i++){
//            cap >> tmp;
//        }
//        cout << "After alignment, video escaping " << gap/33 << " frame"<< endl;
//    }
//}


extern vector<Point2f> convert_keypoints_to_point(vector<KeyPoint> keypoints_ret);


extern void getBinMask( const Mat& comMask, Mat& binMask );


extern Mat grabcut_thumbnail(Mat thumbnail);

extern Point2f get_box_avg_mov_vec(const vector<Point2f> points[2], const vector<uchar> status, obj_box box);
// modify the bounding box to be within the frame
extern Rect legalize_box(Mat im0, Rect bbox);


#endif //MOBILEVIDEONET_CLEAN_GLOBALS_H
