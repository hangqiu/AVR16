//
// Created by nsl on 9/17/16.
//

#include "DeepCam/include/MultiObjTracker.h"
#include "DeepCam/include/Object.h"
#include "DeepCam/include/DeepCam.h"

using namespace std;
using namespace cv;

Size subPixWinSize(10,10), winSize(31,31);
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

std::vector<cv::Point2f> MultiObjTracker::detect_keypoints(cv::Mat im){

//    Ptr<FeatureDetector> detector =FeatureDetector::create(FEATURE_TYPE); //"GFTT", "BRISK"
    const int MAX_COUNT = 500;


    std::vector<cv::Point2f> keypoints_ret;
    cv::goodFeaturesToTrack(im, keypoints_ret, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    cornerSubPix(im, keypoints_ret, subPixWinSize, Size(-1,-1), termcrit);
//    int minHessian = 400;
//    Ptr<cv::BRISK> detector =BRISK::create(minHessian); //"GFTT", "BRISK"
//    // Ptr<FeatureDetector> detector =GFTTDetector::create(); //"GFFT", "BRISK"
    return keypoints_ret;
}



//double scale, char* DatasetPath, Point2f frameTotalOpticalFlow, Point2f totalOpticalFlow, ofstream & latency_file, ofstream& opflow_file)
void MultiObjTracker::TrackMultiObj(){// cv::Mat should be automatically using reference, watch out...

    points[0].clear();
    points[1].clear();

    int start = cvGetTickCount();
    points[0] = detect_keypoints(camera->getPrevGray());
//        vector<cv::Point2f> kypts = detect_keypoints(camera->getPrevGray());
    int mid = cvGetTickCount();
//        points[0] = convert_keypoints_to_point(kypts);
    double detect_time = (double)(mid-start)/ (cvGetTickFrequency()) / 1000000;
    double describe_time= 0;
    Mat described_features;
    int end;
    size_t sizeInBytes;
    // feature discriptor no longer supported by opencv 3.0
    // Seems to have integrated into the detector of function detectionandcompute
    // if (FEATURE_TYPE=="ORB" || FEATURE_TYPE=="BRISK" ){
    //     described_features= describe_keypoints(kypts,camera->getPrevGray());
    //     end = cvGetTickCount();
    //     sizeInBytes= described_features.total() * described_features.elemSize();
    //     describe_time = (double)(end-mid)/ (cvGetTickFrequency()) / 1000000;
    // }
    // // }
    //store keypoints for each box
    //TODO restructure this part..... too messy
    lkt_obj.clear();
    for (int idx = 0;idx<int(camera->getExisting_obj().size()) && (camera->getExisting_obj()[idx].box.left!=YOLO_MARK);idx++){
        LKT_IDX tmp_lkt_vec;
        Rect rect_tmp = Rect(camera->getExisting_obj()[idx].box.left,camera->getExisting_obj()[idx].box.top,camera->getExisting_obj()[idx].box.right-camera->getExisting_obj()[idx].box.left,camera->getExisting_obj()[idx].box.bot-camera->getExisting_obj()[idx].box.top);
        // vector<KeyPoint> tmp_kypts;
        for( int i=0; i < points[0].size(); i++ ){
            if (points[0][i].inside(rect_tmp)){
                tmp_lkt_vec.idx_vec.push_back(i);
                // tmp_kypts.push_back(kypts[i]);
            }
        }
        lkt_obj.push_back(tmp_lkt_vec);
        // Mat tmp_desc = describe_keypoints(tmp_kypts,camera->getPrevGray());
        // size_t sizePerObj = tmp_desc.total() * tmp_desc.elemSize();
        // if (TRACK_EVAL_OUTPUT){
        //     latency_file << sizePerObj << " ";
        // }
    }

//    if (SHOW_IMG){imshow(WIN_NAME, im_0);}

    ///////////////////print existing vector /////////////
    // printf("print existing obj:\n");
    // printExistingObj(camera->getExisting_obj());




    ///////////////////////////////////// CMT processing //////////////////////////
    /////////////////////////////////////  CMT predicting object movement, Tracking the object, update exisiting objects to moved objects //////////////////////////////////////////////////////////////////

//    char key;
//    if (USE_CMT){
//        if (DEBUG) printf("total %d obj, %d cmt\n", int(camera->getExisting_obj().size()), int(cmt_vec.size()));
//        cout << "CMT ... frame " << FrameId << endl;
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
//            // FILE_LOG(logINFO) << "#" << frame_id << ", OBJ: "<< camera->getExisting_obj()[rect_idx].id<<", active: " << cmt_vec[rect_idx].points_active.size();
//        }
//        // vector<obj_box> camera->getMoved_obj()_box;
//        // cout << "im size" << im.rows << " x " << im.cols << "\n";
//        // cout << "Displaying ... frame " << frame_id << endl;
//        key = display_multi(im, cmt_vec, camera->getExisting_obj());
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
//        camera->getMoved_obj() = get_obj_box_from_cmt_vec(cmt_vec,camera->getExisting_obj());
//    }else if (USE_LKT){
        status.clear();
        err.clear();

        // track keypoints
        //Calculate forward optical flow for prev_location
        // time=clock();
        int tck_start = cvGetTickCount();
        int overhead_start = cvGetTickCount();
        calcOpticalFlowPyrLK(camera->getPrevGray(), camera->getCurGray(), points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
        int overhead_end = cvGetTickCount();


//        calcOpticalFlowPyrLK(camera->getPrevGray(), camera->getCurGray(), points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);

        //Calculate backward optical flow for prev_location
        calcOpticalFlowPyrLK(camera->getCurGray(), camera->getPrevGray(), points[1], points_back, status_back, err_back);

        //Traverse vector backward so we can remove points on the fly
        for (int i = points[0].size()-1; i >= 0; i--)
        {
            float l2norm = norm(points_back[i] - points[0][i]);

            bool fb_err_is_large = l2norm > 30; // 30 threshold adopt from open CMT

            if (fb_err_is_large || !status[i] || !status_back[i])
            {
                // points_tracked.erase(points_tracked.begin() + i);

                //Make sure the status flag is set to 0
                status[i] = 0;
            }

        }

        // calculate the total optical flow for background movement subtraction
        // TODO: a more robust way


        frameTotalOpticalFlow.x=0;frameTotalOpticalFlow.y=0;

        int frame_keypt_num=0;
        for (int i = 0; i<points[0].size(); i++){

            if (status[i]){
                frameTotalOpticalFlow = frameTotalOpticalFlow + points[1][i]-points[0][i];
                frame_keypt_num++;
                totalOpticalFlow = totalOpticalFlow + points[1][i]-points[0][i];
//                keypt_num++;
            }
        }
        frameTotalOpticalFlow.x = frameTotalOpticalFlow.x / frame_keypt_num;
        frameTotalOpticalFlow.y = -frameTotalOpticalFlow.y / frame_keypt_num;
//        if (FrameId % (fps/10) == 0){
//            totalOpticalFlow.x = totalOpticalFlow.x / keypt_num;
//            totalOpticalFlow.y = -totalOpticalFlow.y / keypt_num;
//            if (ORIENTATION_SANITY_CHECK){
//                opflow_file << "frame ID: " << FrameId << ", x:"<< totalOpticalFlow.x << ", y:" << totalOpticalFlow.y << ", total flow: " << norm(totalOpticalFlow) << ", direction: " << atan2(totalOpticalFlow.y, totalOpticalFlow.x) << endl;
//            }
//            totalOpticalFlow.x=0;totalOpticalFlow.y=0;
//            keypt_num = 0;
//        }



//        // print circles~~ debug
//        if (SHOW_KYPTS){
//            for( int i = 0; i < points[1].size(); i++ )
//            {
//                if( status[i] ){
//                    circle( camera->getCurGray(), points[0][i], 3, Scalar(255,0,0), -1, 8);
//                    circle( camera->getCurGray(), points[1][i], 3, Scalar(0,255,0), -1, 8);
//                    line( camera->getCurGray(), points[0][i], points[1][i], Scalar(0,0,255));
//
//                }
//            }
//
//        }

        // prepare moved obj
//        camera->getMoved_obj().clear();
//        camera->getMoved_obj() = camera->getExisting_obj();
        camera->setMoved_obj(camera->getExisting_obj());
        for (int moved_idx=0;moved_idx<int(camera->getMoved_obj().size());moved_idx++){
            // camera->getMoved_obj()[moved_idx].id = -1;
            camera->getMoved_obj()[moved_idx].tracked = false;
            Point2f moved_box_mov_vec = get_box_avg_mov_vec(points, status, camera->getMoved_obj()[moved_idx].box);
            if (DEBUG) printf("last moved object: %d,%d,%d,%d, of img %d,%d\n", camera->getMoved_obj()[moved_idx].box.left,camera->getMoved_obj()[moved_idx].box.right, camera->getMoved_obj()[moved_idx].box.top,camera->getMoved_obj()[moved_idx].box.bot, camera->getCurGray().rows, camera->getCurGray().cols);
            camera->getMoved_obj()[moved_idx].box.left += moved_box_mov_vec.x;
            camera->getMoved_obj()[moved_idx].box.top += -moved_box_mov_vec.y;
            camera->getMoved_obj()[moved_idx].box.right += moved_box_mov_vec.x;
            camera->getMoved_obj()[moved_idx].box.bot += -moved_box_mov_vec.y;
            if (DEBUG) printf("moved object: %d,%d,%d,%d, of img %d,%d\n", camera->getMoved_obj()[moved_idx].box.left,camera->getMoved_obj()[moved_idx].box.right, camera->getMoved_obj()[moved_idx].box.top,camera->getMoved_obj()[moved_idx].box.bot, camera->getCurGray().rows, camera->getCurGray().cols);
        }
        // time_end = clock();
        int tck_end = cvGetTickCount();
        double processtime = (double)(tck_end-tck_start)/ (cvGetTickFrequency()) / 1000000;
        if (DEBUG) printf("KLT::processFrame of size (%d,%d) in %f seconds.\n", camera->getCurGray().rows, camera->getCurGray().cols, processtime);
//TODO: reenable the latency log
//        if (TRACK_EVAL_OUTPUT){
//            camera->io->latency_file << detect_time << "+" << describe_time << "+" << processtime << "=" << detect_time+describe_time +processtime << ", size:" << sizeInBytes<< endl;
//        }
        // printf("KLT::processFrame of size (%d,%d) in %f seconds.\n", im_gray.rows, im_gray.cols, sec(time_end-time));
        // show existing obj
//        if (DEBUG) cout << "im size" << camera->getCurGray().rows << " x " << camera->getCurGray().cols << "\n";
//        key = displayExistingObjOnPrevIm(prevIm, camera->getExisting_obj(),0,255,0);
//    }else{
//        camera->getMoved_obj() = camera->getExisting_obj();
//        for (int moved_idx=0;moved_idx<int(camera->getMoved_obj().size());moved_idx++){
//            // camera->getMoved_obj()[moved_idx].id = -1;
//        }
//        // show existing obj
//        key = displayExistingObjOnPrevIm(im, camera->getExisting_obj(),0,255,0);
//    }
    return;
}

const vector<Point2f> *MultiObjTracker::getPoints() const {
    return points;
}

const vector<uchar> &MultiObjTracker::getStatus() const {
    return status;
}

const Point2f &MultiObjTracker::getFrame_total_op_flow() const {
    return frameTotalOpticalFlow;
}

const vector<LKT_IDX> &MultiObjTracker::getLkt_obj() const {
    return lkt_obj;
}

MultiObjTracker::MultiObjTracker(Cam *camera) : camera(camera) {}

MultiObjTracker::~MultiObjTracker() {

}




//vector<Object> MultiObjTracker::get_obj_box_from_cmt_vec(vector<CMT> & cmt_vec, vector<Object> & camera->getExisting_obj()){
//    Point2f vertices[4];
//    vector<Object> camera->getMoved_obj() = camera->getExisting_obj();
//    for (int rect_idx=0;rect_idx < int(cmt_vec.size());rect_idx++){
//        cmt_vec[rect_idx].bb_rot.points(vertices);
////         obj_box tmp_box;
//        // Object tmp_obj;
//
//        // tmp_obj.box.left = vertices[0].x;
//        // tmp_obj.box.bot = vertices[0].y;
//        // tmp_obj.box.right = vertices[2].x;
//        // tmp_obj.box.top = vertices[2].y;
//        // tmp_obj.id = camera->getExisting_obj()[rect_idx].id;
//        // tmp_obj.tracked = false;
//        // camera->getMoved_obj().push_back(tmp_obj);
//
//        camera->getMoved_obj()[rect_idx].box.left = vertices[0].x;
//        camera->getMoved_obj()[rect_idx].box.bot = vertices[0].y;
//        camera->getMoved_obj()[rect_idx].box.right = vertices[2].x;
//        camera->getMoved_obj()[rect_idx].box.top = vertices[2].y;
//        // camera->getMoved_obj()[rect_idx].id = camera->getExisting_obj()[rect_idx].id;
//        camera->getMoved_obj()[rect_idx].tracked = false;
//        // camera->getMoved_obj().push_back(tmp_obj);
//        // printf("Box From CMT: left,%d,top,%d,right,%d,bot,%d\n", tmp_box.left,tmp_box.top,tmp_box.right,tmp_box.bot );
//    }
//    return camera->getMoved_obj();
//}

//bool MultiObjTracker::is_same_dir(double dir1, double dir2){
//    dir1 = fmod(dir1+360,360);
//    dir2 = fmod(dir2+360,360);
//    bool flag1 = abs(dir1 - dir2) < DIRECTION_THRESH;
//    // dis to 180 are very close and one of them is either close to 0 or close to 360
//    bool flag2 = abs(abs(dir1-180)-abs(dir2-180)) < DIRECTION_THRESH && ( dir1 < DIRECTION_THRESH || (360-dir1) < DIRECTION_THRESH );
//    return flag1 || flag2;
//}