///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/******************************************************************************************************************
 ** This sample demonstrates how to use two ZEDs with the ZED SDK, each grab are in a separated thread            **
 ** This sample has been tested with 3 ZEDs in HD720@30fps resolution                                             **
 ** This only works for Linux                                                                                     **
 *******************************************************************************************************************/

//standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <iomanip>

//opencv includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

//ZED Includes
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>
#include <landmark.hpp>
#include <landmark_matchinfo.hpp>
#include <include/Displayer.hpp>
#include <include/globals.hpp>


//our point cloud generator and viewer.
#include "Viewer.hpp"
#include "PointCloud.hpp"

//#include "../ORB_SLAM2/include/System.h"
#include "ORB_SLAM2/include/System.h"

#include "AugmentedVR.hpp"

using namespace sl::zed;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

const sl::zed::ZEDResolution_mode ZED_RES = HD720;
const int NUM_CAMERAS = 2;
int start_frame_id[NUM_CAMERAS] = {2,2};
const sl::zed::SENSING_MODE senseMode = sl::zed::SENSING_MODE::FILL;

// store video or webcam
bool OFFLINE = true;

cv::Mat total_point_cloud;



// TODO: don't delete, later usage
//void grab_run(int x) {
//    while (!stop_signal) {
//        bool res = zed[x]->grab(SENSING_MODE::FILL,  1, 1, 1);
//        if (!res) {
//	        frame_seq[x]++;
//            ZED_Timestamp[x] = zed[x]->getCameraTimestamp();
//            //sl::zed::Mat depthMM = zed[x]->retrieveMeasure(MEASURE::DEPTH);
//            slMat2cvMat(zed[x]->retrieveImage(SIDE::LEFT)).copyTo(SbSResult[x](cv::Rect(0, 0, width, height)));
//            slMat2cvMat(zed[x]->normalizeMeasure(MEASURE::DISPARITY)).copyTo(SbSResult[x](cv::Rect(width, 0, width, height)));
//            // slMat2cvMat(zed[x]->retrieveMeasure_gpu(XYZRGBA)).copyTo(BufferXYZRGBA[x]);
//            slMat2cvMat(zed[x]->retrieveMeasure(XYZRGBA)).copyTo(BufferXYZRGBA[x]);
//        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    }
//    delete zed[x];
//}
//
//void grabZEDFrameOffline(int x) {
//    bool res = zed[x]->grab(SENSING_MODE::FILL, 1, 1, 1);
//    if (!res) {
//	    frame_seq[x]++;
//        ZED_Timestamp[x] = zed[x]->getCameraTimestamp();
//        //sl::zed::Mat depthMM = zed[x]->retrieveMeasure(MEASURE::DEPTH);
//        slMat2cvMat(zed[x]->retrieveImage(SIDE::LEFT)).copyTo(SbSResult[x](cv::Rect(0, 0, width, height)));
//        slMat2cvMat(zed[x]->normalizeMeasure(MEASURE::DEPTH)).copyTo(SbSResult[x](cv::Rect(width, 0, width, height)));
//        slMat2cvMat(zed[x]->retrieveMeasure(XYZRGBA)).copyTo(BufferXYZRGBA[x]);
//        BufferXYZRGBA[x].copyTo(pointcloud[x]);
//
//        depth_mat[x]  = slMat2cvMat(zed[x]->retrieveMeasure(MEASURE::DEPTH));
//        FrameLeft[x] = slMat2cvMat(zed[x]->retrieveImage(SIDE::LEFT));
//        FrameRight[x] = slMat2cvMat(zed[x]->retrieveImage(SIDE::RIGHT));
//
//        BufferXYZRGBA_gpu[x] = zed[x]->retrieveMeasure_gpu(XYZRGBA);
//
//
//
//    	// cout << BufferXYZRGBA[x](cv::Rect(0,0,2,2));
//    }
//}

//cv::Mat getGrayBBox(cv::Mat img, int select_left, int select_top, int select_width, int select_height){
//    // legalize
//    select_left = select_left > 0 ? select_left : 0;
//    select_top = select_top > 0 ? select_top : 0;
//    select_width = select_width+select_left < img.cols ? select_width : img.cols - select_left;
//    select_height = select_height+select_top < img.rows ? select_height : img.rows - select_top;
//
//    cv::Mat img_tmp = img(cv::Rect(select_left, select_top, select_width, select_height));
//    imwrite("img_tmp.jpg",img_tmp);
//    return imread( "img_tmp.jpg", IMREAD_GRAYSCALE );
//}


//
void fetchNUpdateFrameNPointcloud(AugmentedVR** VNode){

    if (FRAME_ID%5==0) // TODO: to be removed, only for debug now, solve the sync issue when zed are in augmentedVR class now
    for (int i = 0; i < NUM_CAMERAS; i++) {
        VNode[i]->updateLastFrame();
    }
    VNode[0]->grabZEDFrameOffline();


    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (OFFLINE){
            while(VNode[i]->frameTS < VNode[1-i]->frameTS){
                VNode[i]->grabZEDFrameOffline();
            }
            // cout << "skipping camera 1 frame ts: " << ZED_Timestamp[0] << endl;
        }

        char wnd_name[21];
        sprintf(wnd_name, "ZED N° %d", i);
//        cv::resize(SbSResult[i], ZED_LRes[i], DisplaySize);

        if (SHOW_IMG) cv::imshow(wnd_name, VNode[i]->ZED_LRes);
        // cout << "copying pont cloud\n";
//            if (SHOW_PC)
//                for (int i = 0; i < NUM_CAMERAS; i++)
//                    BufferXYZRGBA[i].copyTo(pointcloud[i]);
    }
}



///////////////////////////////////////////////////////////main  function////////////////////////////////////////////////////
int main(int argc, char **argv) {

#ifdef WIN32
    std::cout << "Multiple ZEDs are not available under Windows" << std::endl;
    return -1;
#endif


    // parse input
    if (argc != 4) cout << "vidoe 1, video 2, map file";
    string VPath[NUM_CAMERAS];
    int argidx =0;
    for (argidx = 0;argidx < NUM_CAMERAS; argidx++) {
        VPath[argidx] = argv[argidx+1];
        cout << "Video " << argidx << ": " << VPath[argidx] << endl;
    }
    const string mapFile = argv[argidx+1];
    cout << "Map File: " << mapFile << endl;



    string VocFile = "./ORB_SLAM2/Vocabulary/ORBvoc.txt";
    string CalibrationFile = "./CamCalib.yaml";

    InitParams parameters;
    // parameters.mode = PERFORMANCE;
    // parameters.unit = MILLIMETER;
    parameters.mode = MODE::QUALITY; //need quite a powerful graphic card in QUALITY
    parameters.unit = UNIT::METER; // set meter as the OpenGL world will be in meters
    parameters.verbose = 1;
    parameters.coordinate = COORDINATE_SYSTEM::RIGHT_HANDED; // OpenGL's coordinate system is right_handed

    int ZEDConfidence = 85;

    // Create
    AugmentedVR** VNode;
    VNode = new AugmentedVR* [NUM_CAMERAS];
    Displayer* mDisplayer = new Displayer(VNode);


    // Initialization
    for (int i = 0; i < NUM_CAMERAS; i++) {
        VNode[i] = new AugmentedVR(senseMode,i);
        if (argc == 1){
            VNode[i]->initZEDCam(ZED_RES, FPS, parameters, ZEDConfidence);
            OFFLINE = false;
        } 
        else {

            VNode[i]->initZEDCam(VPath[i], start_frame_id[i], parameters, ZEDConfidence);
        }

        // TODO: SLAM can only have one instance. running on different machine for each car would automatically address the issue,
        // For now, just instantiate one SLAM and point other pointers to the same slam.
        // Alternatively calling SLAM from diferent cam instance will yeild bad result presumably.
        if (i==0)
            VNode[i]->initSLAMStereo(VocFile, CalibrationFile,true, mapFile);
        // TODO share the vocabulaty, load and share the map
        else VNode[i]->setMSLAM(VNode[0]->getMSLAM());
    }

    mDisplayer->init();

//    if (!OFFLINE){ // TODO restore the realtime process part later
//        //Create both grabbing thread with the camera number as parameters
//        for (int i = 0; i < NUM_CAMERAS; i++)
//            thread_vec.push_back(new std::thread(grab_run, i));
//    }

    // wait till both thread get it going.. for now.. need sync mechanism
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

//TODO: reenble this part with YOLO
//// detect object / features / landmarks (static)
//// initialize front car bounding box
//    int select_width = 200;
//    int select_height = 100;
//    int select_left = 520;
//    int select_top = 500;
//    cv::Mat img_object = getGrayBBox(FrameLeft[1],select_left-50, select_top-50, select_width+100, select_height+100);
//
//// parked car as landmark
////    int lm_width = 120;
////    int lm_height = 70;
////    int lm_left = 292;
////    int lm_top = 419;
//    int lm_width = 70;
//    int lm_height = 100;
//    int lm_left = 764;
//    int lm_top = 398;
//
//// extract landmarks
////    extract the front car
//    cv::Mat img_lm = getGrayBBox(FrameLeft[1],lm_left, lm_top, lm_width, lm_height);
//    cv::Mat shift_region(lm_height, lm_width, CV_32FC4, Scalar(3,0,0,0));
//    // maneupulate the point cloud first to double check if index is correct
//    cv::Mat pc_lm;
//    pointcloud[1](cv::Rect(lm_left, lm_top, lm_width, lm_height)).copyTo(pc_lm);
////    pc_object += shift_region;
//
//    landmark lm(img_lm, pc_lm, lm_left,lm_top,lm_width,lm_height);
    sl::zed::Mat total_point_cloud_gpu;
    char key = ' ';
    //loop until 'q' is pressed
    ////////////////////////////////////////////////////////////// main loop/////////////////////////////////////////////////
    while (key != 'q') {
        FRAME_ID++;
        //Resize and imshow
        cout << "FrameID: " << FRAME_ID << endl;

        fetchNUpdateFrameNPointcloud(VNode);
        //compare Timestamp between both camera (uncomment following line)
        // for (int i = 0; i < NUM_CAMERAS; i++) std::cout << " Timestamp " << i << ": " << ZED_Timestamp[i] << std::endl;
//        if (DEBUG) std::cout << " Timestamp: " << frame_ts[0] << " - " << frame_ts[1] <<  " = "<<  frame_ts[0] - frame_ts[1] << std::endl;

        //get the Key
        key = cv::waitKey(20);
        // pause and resume
        mDisplayer->processKey(key);

        VNode[0]->trackCam();
        VNode[1]->trackCam();
        if (VNode[0]->CamMotionMat.empty() || VNode[1]->CamMotionMat.empty()) continue;

        VNode[0]->calcPCMotionVec();
        VNode[1]->calcPCMotionVec();

        VNode[0]->filterPCMotion();
        VNode[1]->filterPCMotion();

        cv::Mat Trc, trc, RxPC, transRxPC;

        Trc =  VNode[1]->calcRelaCamPos(VNode[0]->CamMotionMat);
        trc = Trc.rowRange(0,3).col(3);
        RxPC = VNode[0]->pointcloud;
        transRxPC = VNode[1]->transfromRxPCtoMyFrameCoord(trc, RxPC);

    // Point Cloud Stiching
        if (SHOW_PC) {
//            mDisplayer->showDynamicPC();
//            mDisplayer->showPC(VNode[1]->pointcloud);
//            mDisplayer->showPC(VNode[1]->DynamicPC);
            mDisplayer->showMergedPC(transRxPC);
        }
        cout << endl;
    }

    //out --> tells both thread to finish
//    stop_signal = true;

    //end of thread --sync
//    if (!OFFLINE)
//        for (auto it : thread_vec) it->join();

//    myfile.close();
//    logfile.close();
    return 0;
}







// Feature / Object Extraction (dynamic)




// get the distance vector
//        TODO: reenable
//        cv::Mat distVector;
//        if (LANDMARK_DISTVECTOR){
//
//            cv::Mat img_scene = getGrayBBox(FrameLeft[0],0,0,width,height);
////        cv::Mat img_scene = getGrayBBox(FrameLeft[1],0,0,width,height);
////        float distance = getDistanceVector_Depth(img_object,img_scene,select_left,select_top,select_width,select_height);
//            distVector = getDistanceVector_Landmark(lm,img_scene);
//            cout << "distance vector mat:" << distVector << endl;
//        }

//bool valid_bbox(int x){
//
//}
// TODO: don't delete, maybe useful later
//cv::Mat getDistanceVector_Landmark(landmark lm, cv::Mat img_scene){
//    landmark_matchinfo matched_obj = find_obj_in_second_scene(lm.bbox, img_scene);
//    // matched_obj.obj, scene with the same index are matched keypoints
//    // TODO: make the parameter match info only. smarter algorithm to get the distance, fornow it's only onepoint
//    int xInObjPC = matched_obj.obj[0].x;
//    int yInObjPC = matched_obj.obj[0].y;
//    int xInScenePC = matched_obj.scene[0].x;
//    int yInScenePC = matched_obj.scene[0].y;
////    cv::Scalar vectorInObjPC = lm.PointCloud(cv::Rect(xInObjPC,yInObjPC,1,1));
//    cv::Mat vectorInObjPC = lm.PointCloud(cv::Rect(xInObjPC,yInObjPC,1,1));
//    cv::Mat vectorInScenePC = pointcloud[0](cv::Rect(xInScenePC,yInScenePC,1,1)); // use the scene PC
//    //TODO: rotation matrix
//    cv::Mat distVector = vectorInObjPC - vectorInScenePC;
//    return distVector;
//}
//
//
//float getDistanceVector_Depth(cv::Mat img_object,cv::Mat img_scene, int& select_left, int& select_top, int& select_width, int& select_height){
//    // the depth way: only one car in the front in the same lane
//    // int test_x = 1031; int test_y=320;
//    // int x = 718; int y = 464;// front car
//    // myfile << depth_mat[1](cv::Rect(x, y, 1, 1)) << endl;
//    float distance = *(float*)depth_mat[1](cv::Rect(select_left+select_width/2, select_top+select_height/2, 1, 1)).data;
////        distance  /= 1000;
////        double distance[3] = pointcloud[1](cv::Rect(select_left+select_width/2, select_top+select_height/2, 1, 1));
//    cout << depth_mat[1](cv::Rect(select_left+select_width/2, select_top+select_height/2, 1, 1)) << ", ";
//    cout << distance<< endl;
//
//
//    // Tracking the front car
//    if (FRAME_ID % TRACK_FREQ == 0){
//
//
////        cv::Mat img2_tmp = FrameLeft[1];
////        imwrite("img2.jpg",img2_tmp);// something is wrong (dimension) with the container, need to write and read again
////        cv::Mat img_scene = imread( "img2.jpg", IMREAD_GRAYSCALE );
//
//        landmark_matchinfo matched_obj = find_obj_in_second_scene(img_object, img_scene);
//        select_left = matched_obj.x_dst;
//        select_top = matched_obj.y_dst;
//        select_width = matched_obj.width_dst;
//        select_height = matched_obj.height_dst;
//        // update bbox
//
////        select_width = scene_corners[1].x-scene_corners[0].x;select_height=scene_corners[3].y-scene_corners[0].y;
////            img_object = getGrayBBox(FrameLeft[1],select_left, select_top, select_width, select_height);
////        img1_tmp = FrameLeft[1](cv::Rect(select_left-50, select_top-50, select_width+100, select_height+100));
////        imwrite("img1.jpg",img1_tmp);
////        img_object = imread( "img1.jpg", IMREAD_GRAYSCALE );
//    }
//    // TODO: the common landmark way.
////        // extracting the object
//////        cv::Mat img1_tmp = ZED_LRes[1](cv::Rect(250, 100, 100, 100));
//////        cv::Mat img2_tmp = ZED_LRes[0](cv::Rect(0, 0, 360, 404));
////        cv::Mat img1_tmp = FrameLeft[1](cv::Rect(select_left, select_top, select_width, select_height));
////        cv::Mat img2_tmp = FrameLeft[0](cv::Rect(0, 0, width, height));
////        imwrite("img1.jpg",img1_tmp);
////        imwrite("img2.jpg",img2_tmp);// something is wrong (dimension) with the container, need to write and read again
////        cv::Mat img_object = imread( "img1.jpg", IMREAD_GRAYSCALE );
////        cv::Mat img_scene = imread( "img2.jpg", IMREAD_GRAYSCALE );
////
////        landmark lm(img_object, pc_object, select_left,select_top,select_width,select_height);
////
////        // Feature / object matching
//////TODO: return matchign bounding box, if no quality match, return NULL;
////        find_obj_in_second_scene(img_object, img_scene);
//    return distance;
//}


//landmark_matchinfo find_obj_in_second_scene(cv::Mat img_object, cv::Mat img_scene)
//{
//
//    // cvtColor(img_object, img_object, cv::COLOR_RGB2GRAY);
//    // cvtColor(img_scene, img_scene, cv::COLOR_RGB2GRAY);
//
//    // Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
//    // Mat img_scene = imread( argv[2], IMREAD_GRAYSCALE );
//
//    //-- Localize the object
//    std::vector<Point2f> obj;
//    std::vector<Point2f> scene;
//    std::vector<Point2f> obj_corners(4);
//    std::vector<Point2f> scene_corners(4);
//
////    if( !img_object.data || !img_scene.data )
////  { std::cout<< " --(!) Error reading images " << std::endl; return NULL; }
//    //-- Step 1: Detect the keypoints and extract descriptors using SURF
//    int minHessian = 400;
//    Ptr<SURF> detector = SURF::create( minHessian );
//
//    // SurfFeatureDetector detector( minHessian );
//    std::vector<KeyPoint> keypoints_object, keypoints_scene;
//    cv::Mat descriptors_object, descriptors_scene;
//    detector->detectAndCompute( img_object, cv::Mat(), keypoints_object, descriptors_object );
//    detector->detectAndCompute( img_scene, cv::Mat(), keypoints_scene, descriptors_scene );
//    //-- Step 2: Matching descriptor vectors using FLANN matcher
//    FlannBasedMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_object, descriptors_scene, matches );
//    double max_dist = 0; double min_dist = 100;
//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < descriptors_object.rows; i++ )
//  { double dist = matches[i].distance;
//      if( dist < min_dist ) min_dist = dist;
//      if( dist > max_dist ) max_dist = dist;
//  }
//    if(DEBUG){
//        printf("-- Max dist : %f \n", max_dist );
//        printf("-- Min dist : %f \n", min_dist );
//    }
//    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< DMatch > good_matches;
//    for( int i = 0; i < descriptors_object.rows; i++ )
//  { if( matches[i].distance < 3*min_dist )
//     { good_matches.push_back( matches[i]); }
//  }
//    cv::Mat img_matches;
//    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
//               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    for( size_t i = 0; i < good_matches.size(); i++ )
//  {
//      //-- Get the keypoints from the good matches
//      obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//      scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//  }
//    cv::Mat H = findHomography( obj, scene, RANSAC );
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
//    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
//  perspectiveTransform( obj_corners, scene_corners, H);
//  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
//  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  //-- Show detected matches
//  imshow( "Good Matches & Object detection", img_matches );
//  if (DEBUG) imwrite("./match.jpg", img_matches);
//  // waitKey(0);
//    int select_width,select_height,select_left, select_top;
//    if (!scene_corners.empty()){
//        int sum_x = 0; int sum_y = 0; int sum_w = 0; int sum_h = 0;
//        for (int ii=0;ii<4;ii++){
//            sum_x += scene_corners[ii].x;
//            sum_y += scene_corners[ii].y;
////            sum_w += abs(scene_corners[ii].x - scene_corners[(ii-1) % 4].x);
////            sum_x += scene_corners[ii].x;
////            sum_y += scene_corners[ii++].y;
////            sum_h += abs(scene_corners[ii].y - scene_corners[(ii-1) % 4].y);
//        }
//        sum_w = scene_corners[1].x - scene_corners[0].x + scene_corners[2].x - scene_corners[3].x;
//        sum_h = scene_corners[2].y - scene_corners[1].y + scene_corners[3].y - scene_corners[0].y;
//        select_left = sum_x / 4 - sum_w / 4;
//        select_top = sum_y / 4 - sum_h / 4;
//        select_width = sum_w / 2; select_height = sum_h / 2;
//    }
//  landmark_matchinfo ret(img_object, H, select_left,select_top,select_width, select_height,obj,scene);
//  return ret;
////  return scene_corners;
//}
//
//void shiftPC(cv::Mat pc1,cv::Scalar vec){
//    cv::Mat shift(pc1.rows, pc1.cols, CV_32FC4, vec);
//    pc1+=shift;
//}
//
//void manipulatePC(cv::Mat distVector){
//    float* distvec = (float*) distVector.data;
//    Scalar vec = Scalar(distvec[0],distvec[1],distvec[2],0);
//    cout << "manipulate vector: " <<  vec << endl;
//    cv::Mat shift(pointcloud[1].rows, pointcloud[1].cols, CV_32FC4, vec);
//    pointcloud[0]+=shift;
//}
//
//void manipulatePC(float distance){
//    // cout << "shifting pont cloud\n";
//    cv::Mat shift(pointcloud[1].rows, pointcloud[1].cols, CV_32FC4, Scalar(0,0,-(distance+3),0));
//    pointcloud[0]+=shift;
//
//    hconcat(pointcloud[0],pointcloud[1],total_point_cloud);
//
//    cv::Mat shift_pc(pointcloud[1].rows, pointcloud[1].cols, CV_32FC4, Scalar(0,10,0,0));
//    pointcloud[1]-=shift_pc;
//
//    hconcat(total_point_cloud,pointcloud[1],total_point_cloud);
//
////        cv::Mat M(2,2, CV_32FC4, Scalar(1,0,0,NAN));
////     cout << "M = " << endl << " " << M << endl << endl;
//
//
//    // pointcloud[0](cv::Rect(select_left, select_top, select_width, select_height)) += shift_region;
//    // cout << pointcloud[0](cv::Rect(select_left+select_width-1,select_top+select_height-1,2,2)) << endl;
//
//    //       select_left = 656;
//    // select_top = 185;
//
//    // cout << pointcloud[1](cv::Rect(select_left+select_width-1,select_top+select_height-1,2,2)) << endl;
//    // myfile << pointcloud[1](cv::Rect(select_left+select_width-1,select_top+select_height-1,1,1));
//    //       int select_left = 460;
//    // int select_top = 396;
//    // int select_width = 20;
//    // int select_height = 20;
//    //       // cv::Mat shift(total_point_cloud.rows,total_point_cloud.cols, CV_32FC4, Scalar(2,0,0,0));
//    // cv::Mat shift_region(select_height, select_width, CV_32FC4, Scalar(3,0,0,0));
//    //       pointcloud[1](cv::Rect(select_left, select_top, select_width, select_height)) += shift_region;
//    //       cout << total_point_cloud(cv::Rect(select_left,select_top,2,2));
//    // watchout, here it's height, width, whereas cv::rect(,,width,height)
//    // cout << shift;
//    // cout << total_point_cloud(cv::Rect(640, 0, 2, 1));
//
//    // cout << total_point_cloud(cv::Rect(75, 330, 2, 2));
//    // total_point_cloud(cv::Rect(200, 330, 80, 80)) += shift;
//    // cout << total_point_cloud(cv::Rect(583, 126, 2, 2));
//    // total_point_cloud(cv::Rect(583, 126, 2, 2)) += shift;
//
//    // cv::Mat tmp = total_point_cloud.colRange(340,380).rowRange(620,660);
//    //      if (DEBUG){
//    //      	cout 	<< "PC dims:" << tmp.rows
//    //      			<< ", "<< tmp.cols
//    //      			<< ", "<< tmp.channels() <<  endl;
//    // cout << tmp;
//    //      }
//    // float* buffer_ptr = (float*)tmp.data;
//    // 			// uchar* tmp2 = total_point_cloud.data;
//    // 			// cout << *(total_point_cloud.data);
//    // 			// cout << *total_point_cloud.ptr(1,1);
//    // int index = 0;
//    // // cout << tmp << endl;
//    // while( index < tmp.rows*tmp.cols*tmp.channels()){
//    //     buffer_ptr[index++]+=0.0;
//    //     buffer_ptr[index++]+=0.0;
//    //     buffer_ptr[index++]+=0.0;
//    //     buffer_ptr[index++]+=0.0;
//    // }
//    // cout << tmp << endl;
//}

//void process_key(char key){
//    if (key == 'p' || PAUSE_FLAG) pause_step_resume();
//    if (key == 's') save_img();
//}
//
//void pause_step_resume(){
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
//        if (new_key == 's') {
//            save_img();
//        }
//    }
//    if (DEBUG) cout << "resuming....\n";
//    PAUSE_FLAG = false;
//}
//
//void save_img(){
//    char tmp_str[50];
//    int x=0;
//
//    cout << "press the number of camera you want to save, a for all\n";
//    char new_key;
//    while(new_key = cv::waitKey(20)){
//        if (new_key == '0' || new_key == 'a'){
//            x = 0;
//            sprintf(tmp_str, "./cam%d_frame%d_ts%ld_left.png", x, frame_seq[x], frame_ts[x]);
//            imwrite(tmp_str,slMat2cvMat(zed[x]->retrieveImage(SIDE::LEFT)));
//            cout << "saved cam 0 as " << tmp_str << endl;
//        }
//        if (new_key == '1' || new_key == 'a'){
//            x = 1;
//            sprintf(tmp_str, "./cam%d_frame%d_ts%ld_left.png", x, frame_seq[x], frame_ts[x]);
//            imwrite(tmp_str,slMat2cvMat(zed[x]->retrieveImage(SIDE::LEFT)));
//            cout << "saved cam 1 as " << tmp_str << endl;
//        }
//        if (new_key == 'r'){
//            cout << "exiting save_img mode\n";break;
//        }
//    }
//
//}