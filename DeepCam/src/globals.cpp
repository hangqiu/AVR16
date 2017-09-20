//
// Created by nsl on 9/18/16.
//

#include "DeepCam/include/globals.h"

bool PAUSE_FLAG = false;
bool FLOW_CONSISTENCY_CHECK = false;
bool DEBUG = false;
const bool SHOW_IMG = true;
const bool USE_LKT = true;


double STOP_CENTROID_THRESH = 4;

double STOP_SCALE_THRESH = 0.8;
bool ORIENTATION_SANITY_CHECK = false;
// bool ORIENTATION_SANITY_CHECK = true;
bool TRACK_EVAL_OUTPUT = true;
bool YOLO_DETECTION_OUTPUT = false; // detection output frequency is the same with yolo_freq
bool GRABCUT_FLAG = false;
bool DECOUPLE_TO_FRAMES = false;
bool MOBILE_CAM = false;
bool BOX_AREA_FILTER = false;
bool SHOW_KYPTS_THUMBNAIL = false;
bool SHOW_KYPTS = false;

//const int YOLO_MARK = -1000; // already defined in ../YOLO/detection.c
double MOVE_CENTROID_THRESH = 250; // CAN MOVE ANYWHERE FUNCTION WILL FIND CLOSEST
const double MOVE_SCALE_THRESH = 0.4; // CAN CHANGE QUITE A BIT
const double KEYPOINT_RATIO_THRESH = 0.3;

const int BOUNDARY_THRESH = 20; //pixel
const double BOX_AREA_THRESH = 4000; // boxes less than box area will not be considered...

const double DIRECTION_THRESH = 10;
// obj flag
// const string STOP="Stop";
// const string MOVE="Move";
// obj status
const string ENTER="Enter";
const string MOVE = "Moving";
const string EXIT = "Exit";
const string UNKNOWN = "Unknown";
// obj view
const string FRONT = "Front";
const string MID = "Mid";


const string REAR = "Rear";
const double EVICT_TRHESH = 0.5; // seconds
const double OUTPUT_FREQ = 0.1; // seconds output cld and object per second for moving object

const int FRAME_PER_PROC = 1;// process per FRAME_PER_PROC number of frames
int YOLO_FREQ = 1;


int START_FRAME = 0;

const string WIN_NAME = "Gotcha";
const int RESIZEROWS = 360;

// double get_centroid_distance(struct obj_box box1, struct obj_box box2){
double get_dist(int centroid1_x, int centroid1_y, int centroid2_x, int centroid2_y){
    return sqrt((centroid1_x-centroid2_x)*(centroid1_x-centroid2_x) + (centroid1_y-centroid2_y)*(centroid1_y-centroid2_y));
}



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


vector<Point2f> convert_keypoints_to_point(vector<KeyPoint> keypoints_ret){
    vector<Point2f> points_ret;
    for (size_t i = 0; i < keypoints_ret.size(); i++)
    {
        points_ret.push_back(keypoints_ret[i].pt);
    }
    return points_ret;
}


void getBinMask( const Mat& comMask, Mat& binMask )
{
    if( comMask.empty() || comMask.type()!=CV_8UC1 )
        CV_Error( CV_StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
        binMask.create( comMask.size(), CV_8UC1 );
    binMask = comMask & 1;
}

Mat grabcut_thumbnail(Mat thumbnail){
    ////////////////////////// grabcut //////////////////////
    Mat mask, binMask, res;
    mask.create( thumbnail.size(), CV_8UC1);
    Mat bgdModel, fgdModel;
    // Rect rect = Rect( Point(10,10), Point(thumbnail.cols-10,thumbnail.rows-10) );
    Rect rect;
    int margin = 1;
    rect.x = margin;
    rect.y = margin;
    rect.width = thumbnail.cols-margin*2;
    rect.height = thumbnail.rows-margin*2;
    // printf("give foreground: x,y,width,height, %d/,%d,%d/%d,%d/%d\n", rect.x,rect.y,rect.width, thumbnail.cols, rect.height,thumbnail.rows);
    // cout << "After grabcut\n";
    mask.setTo( GC_PR_BGD );
    // setRectInMask();
    (mask(rect)).setTo( Scalar(GC_PR_FGD) );
    grabCut( thumbnail, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT );

    getBinMask( mask, binMask );
    thumbnail.copyTo( res, binMask );

    // imshow("After grabcut", res);
    return res;
}


Point2f get_box_avg_mov_vec(const vector<Point2f> points[2], const vector<uchar> status, obj_box box){
    Rect rect = Rect(box.left,box.top,box.right-box.left,box.bot-box.top);
    Rect rect_top_left = Rect(box.left,box.top,(box.right-box.left)/2,(box.bot-box.top)/2);
    Rect rect_top_right = Rect(box.left+(box.right-box.left)/2,box.top,(box.right-box.left)/2,(box.bot-box.top)/2);
    Rect rect_btm_left = Rect(box.left,box.top+(box.bot-box.top)/2,(box.right-box.left)/2,(box.bot-box.top)/2);
    Rect rect_btm_right = Rect(box.left+(box.right-box.left)/2,box.top+(box.bot-box.top)/2,(box.right-box.left)/2,(box.bot-box.top)/2);
    Point2f total_mv_vec;

    total_mv_vec.x=0;
    total_mv_vec.y=0;
    double points_num = 0;

    bool check_tl=false, check_tr=false, check_bl=false, check_br=false;
    for (int i=0;i<points[1].size(); i++){
        if (status[i] && points[1][i].inside(rect)){
            points_num += 1;
            // point moving vector
            Point2f pt_mov_vec=points[1][i] - points[0][i];
            // printf("points (%f,%f) moved to (%f,%f), mov_vec %f\n",points[0][i].x, points[0][i].y, points[1][i].x, points[1][i].y, norm(pt_mov_vec));
            total_mv_vec = total_mv_vec + pt_mov_vec;
            // check if the content in the box are all moving, method, check if there is optical flow in 4 sub region of the box
            if (FLOW_CONSISTENCY_CHECK && norm(pt_mov_vec) > STOP_CENTROID_THRESH){
                if (points[1][i].inside(rect_top_left)) check_tl=true;
                if (points[1][i].inside(rect_top_right)) check_tr=true;
                if (points[1][i].inside(rect_btm_left)) check_bl=true;
                if (points[1][i].inside(rect_btm_right)) check_br=true;
                // printf("points (%f,%f) in box [%d,%d,%d,%d]\n",points[1][i].x, points[1][i].y, box.left,box.right, box.top, box.bot);
            }
        }
    }
    Point2f avg_mv_vec(0,0);
    if ( points_num != 0 && (!FLOW_CONSISTENCY_CHECK || ((check_tl&&check_br) && (check_tr && check_bl)) )){
        avg_mv_vec.x = total_mv_vec.x / double(points_num);
        avg_mv_vec.y = -total_mv_vec.y / double(points_num);// transform the coordinates, from left to right, top to bot, matrix coordinates is x(0->inf), y->(0->inf), real coordinate is x(-inf->inf), y(inf->-inf)
        if (DEBUG) printf("New Box {top left][%d,%d] moved %f / %f = %f towards %f\n", box.left, box.top ,norm(total_mv_vec),points_num,norm(avg_mv_vec), atan2(avg_mv_vec.y, avg_mv_vec.x)*180/PI);
    }
    return avg_mv_vec;
}

// modify the bounding box to be within the frame
Rect legalize_box(Mat im0, Rect bbox){
    // printf("Oringal: %d,%d,%d,%d\n", rect_tmp.x,rect_tmp.y,rect_tmp.x+rect_tmp.width,rect_tmp.y+rect_tmp.height );
    if (bbox.x<0){
        bbox.width += bbox.x;
        bbox.x = 0;
    }
    if (bbox.y<0){
        bbox.height += bbox.y;
        bbox.y = 0;
    }
    if (bbox.y+bbox.height > im0.rows){
        bbox.height =  im0.rows - bbox.y;
    }
    if (bbox.x+bbox.width > im0.cols){
        bbox.width = im0.cols- bbox.x;
    }
    return bbox;
    // printf("Legalized: %d,%d,%d,%d\n", rect_tmp.x,rect_tmp.y,rect_tmp.x+rect_tmp.width,rect_tmp.y+rect_tmp.height );
}

