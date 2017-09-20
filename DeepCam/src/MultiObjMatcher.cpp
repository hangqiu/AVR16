//
// Created by nsl on 9/17/16.
//

#include "DeepCam/include/MultiObjMatcher.h"




void MultiObjMatcher::updateStaleness(){
    for (int moved_idx = 0;moved_idx<int(camera->getMoved_obj().size())&&(camera->getMoved_obj()[moved_idx].box.left!=YOLO_MARK);moved_idx++){
        //TODO: remove cmt vec not matched and obj_bb_box.
        // if (camera->getMoved_obj()[moved_idx].id==-1){
        if (camera->getMoved_obj()[moved_idx].tracked==false){
            if (camera->getExisting_obj()[moved_idx].alive){
                if (DEBUG) printf("Object %d: Lost track.\n",camera->getExisting_obj()[moved_idx].id);

            }
//            camera->getExisting_obj()[moved_idx].alive=false;
            camera->getExisting_obj()[moved_idx].setAlive(false);
            camera->getExisting_obj()[moved_idx].setEvict(true);

            // should update the box position according to keypoints in case in future recognized again
            Point2f exist_box_mov_vec = get_box_avg_mov_vec(multiObjTracker->getPoints(), multiObjTracker->getStatus(), camera->getExisting_obj()[moved_idx].box);
            exist_box_mov_vec.y = -exist_box_mov_vec.y;
            Point2f left_top(camera->getExisting_obj()[moved_idx].box.left,camera->getExisting_obj()[moved_idx].box.top);
            Point2f right_bot(camera->getExisting_obj()[moved_idx].box.right,camera->getExisting_obj()[moved_idx].box.bot);
            left_top += exist_box_mov_vec;
            right_bot += exist_box_mov_vec;
            camera->getExisting_obj()[moved_idx].box.left = left_top.x < 0 ? 0:left_top.x;
            camera->getExisting_obj()[moved_idx].box.top = left_top.y < 0 ? 0:left_top.y;
            camera->getExisting_obj()[moved_idx].box.right = right_bot.x > camera->getCols()? camera->getCols(): right_bot.x ;
            camera->getExisting_obj()[moved_idx].box.bot = right_bot.y > camera->getRows()? camera->getRows():right_bot.y ;

            // invalid anymore,
            // shouldn't erase, erase will be recognized again
            // camera->getExisting_obj().erase(camera->getExisting_obj().begin()+moved_idx);
            // cmt_vec.erase(cmt_vec.begin()+moved_idx);
        }
    }
}
void MultiObjMatcher::matchingMovedObj(){
    for (int new_idx = 0;new_idx<int(camera->getNew_obj().size())&&(camera->getNew_obj()[new_idx].box.left!=YOLO_MARK);new_idx++){

        Point2f new_box_mov_vec = get_box_avg_mov_vec(multiObjTracker->getPoints(), multiObjTracker->getStatus(), camera->getNew_obj()[new_idx].box);



        if (camera->getNew_obj()[new_idx].id!=-1 && check_movement(new_box_mov_vec,multiObjTracker->getFrame_total_op_flow())){
            // box is moving
            double min_centroid_dist = 10000;
            int matched_idx = -1;
            // Point2f min_centroid;
            int new_centroid_x = (camera->getNew_obj()[new_idx].box.left+camera->getNew_obj()[new_idx].box.right)/2;
            int new_centroid_y = (camera->getNew_obj()[new_idx].box.top+camera->getNew_obj()[new_idx].box.bot)/2;
            for (int moved_idx = 0;moved_idx<int(camera->getMoved_obj().size())&&(camera->getMoved_obj()[moved_idx].box.left!=YOLO_MARK);moved_idx++){
                // should not check obj_bb_box if moving first, here is the only chance for a stopped car to be recognized again.
                // double tmp_dist = norm(new_centroid-exist_centroid);
                double tmp_dist = get_centroid_distance(camera->getMoved_obj()[moved_idx].box,camera->getNew_obj()[new_idx].box);
                double tmp_scale = get_scale_distance(camera->getMoved_obj()[moved_idx].box,camera->getNew_obj()[new_idx].box);
                if ( tmp_dist < min_centroid_dist && same_box_test(tmp_dist,tmp_scale,MOVE_CENTROID_THRESH, MOVE_SCALE_THRESH)){
                    if (USE_LKT){
                        double total = 0;
                        double match = 0;
                        for ( int i=0;i<multiObjTracker->getLkt_obj()[moved_idx].idx_vec.size();i++){ //TODO remove usage of lkt_obj
                            if (multiObjTracker->getStatus()[multiObjTracker->getLkt_obj()[moved_idx].idx_vec[i]]){
                                total = total+1.0;
                                Rect rect_tmp = Rect(camera->getNew_obj()[new_idx].box.left,camera->getNew_obj()[new_idx].box.top,camera->getNew_obj()[new_idx].box.right-camera->getNew_obj()[new_idx].box.left,camera->getNew_obj()[new_idx].box.bot-camera->getNew_obj()[new_idx].box.top);
                                if (multiObjTracker->getPoints()[1][multiObjTracker->getLkt_obj()[moved_idx].idx_vec[i]].inside(rect_tmp) ){
                                    match = match+1.0;
                                }
                            }
                        }
                        if ( (total >=3 && match / total > KEYPOINT_RATIO_THRESH) || total <3){
                            min_centroid_dist = tmp_dist;
                            // min_centroid = exist_centroid;
                            matched_idx = moved_idx;
                        }else{
                            if (DEBUG) printf("Yolo box #%d, keypoints not matching existing Obj %d, total: %f, match: %f\n", new_idx, camera->getExisting_obj()[moved_idx].id,  total,match);
                        }
                    }else{
                        min_centroid_dist = tmp_dist;
                        // min_centroid = exist_centroid;
                        matched_idx = moved_idx;
                    }
                }else{
                    if (DEBUG) printf("Yolo box #%d, box distance not matching to existing Obj %d, dist: %f, scale_dist %f \n", new_idx, camera->getExisting_obj()[moved_idx].id, tmp_dist, tmp_scale);
                }
            }
            if (matched_idx!=-1){
                camera->getNew_obj()[new_idx].id = camera->getExisting_obj()[matched_idx].id;
                // mark camera->getMoved_obj()_box matched by giving the object id that it has. by marking the tracked flag true now
                // camera->getMoved_obj()[matched_idx].id = camera->getExisting_obj()[matched_idx].id;
                camera->getMoved_obj()[matched_idx].tracked = true;

                // update moving direction
                int exist_centroid_x = (camera->getExisting_obj()[matched_idx].box.left+camera->getExisting_obj()[matched_idx].box.right)/2;
                int exist_centroid_y = (camera->getExisting_obj()[matched_idx].box.top+camera->getExisting_obj()[matched_idx].box.bot)/2;
                // camera->getExisting_obj()[matched_idx].displacement = get_dist(new_centroid_x,new_centroid_y,exist_centroid_x,exist_centroid_y);
                // camera->getExisting_obj()[matched_idx].direction = atan2(new_centroid_y-exist_centroid_y,new_centroid_x - exist_centroid_x) *180 / PI;
                if (MOBILE_CAM){
                    camera->getExisting_obj()[matched_idx].displacement = norm(new_box_mov_vec - multiObjTracker->getFrame_total_op_flow());
                    camera->getExisting_obj()[matched_idx].direction = atan2((new_box_mov_vec - multiObjTracker->getFrame_total_op_flow()).y, (new_box_mov_vec - multiObjTracker->getFrame_total_op_flow()).x) *180 / PI;
                }else{
                    camera->getExisting_obj()[matched_idx].displacement = norm(new_box_mov_vec);
                    camera->getExisting_obj()[matched_idx].direction = atan2(new_box_mov_vec.y, new_box_mov_vec.x) *180 / PI;
                }


                if (DEBUG) printf("moving from (%d,%d) to (%d,%d), dist: %f, dir: %f\n", exist_centroid_x, exist_centroid_y, new_centroid_x, new_centroid_y,  camera->getExisting_obj()[matched_idx].displacement,camera->getExisting_obj()[matched_idx].direction );
                // return -pi to pi
                //         90
                // -/+180--------0
                //        -90

                // if (new_centroid_x < exist_centroid_x){
                //     camera->getExisting_obj()[matched_idx].direction+=180;
                // }
                // camera->getExisting_obj()[matched_idx].direction+=90;
                // camera->getExisting_obj()[matched_idx].displacement = norm(new_centroid-exist_centroid);
                // update TO BE A moving object
                if (camera->getExisting_obj()[matched_idx].displacement<STOP_CENTROID_THRESH){ // TODO, check if need to replace it with check_movement function here as well
                    camera->getExisting_obj()[matched_idx].alive = false;
                    camera->getExisting_obj()[matched_idx].evict = false;
                    camera->getExisting_obj()[matched_idx].evict_count = 0;
                    continue;
                }else{
                    if (DEBUG) printf("Object %d moved %f towards %f \n", camera->getExisting_obj()[matched_idx].id,  camera->getExisting_obj()[matched_idx].displacement,camera->getExisting_obj()[matched_idx].direction);
                    // printf("    Old centroid: %d,%d   New centroid: %d,%d\n", exist_centroid_x,exist_centroid_y, new_centroid_x, new_centroid_y);
                    camera->getExisting_obj()[matched_idx].alive = true;
                    camera->getExisting_obj()[matched_idx].evict = false;
                    camera->getExisting_obj()[matched_idx].evict_count = 0;
                    camera->getExisting_obj()[matched_idx].move_count++;
                    if (!check_boundary(camera->getCols(), camera->getRows(), camera->getExisting_obj()[matched_idx].box)){
                        // too close to the boundary : either enter or exit
                        if ( camera->getExisting_obj()[matched_idx].status.compare(ENTER) ==0 ){
                            // keep status of enter
                        }else if (camera->getExisting_obj()[matched_idx].status.compare(MOVE) ==0 ){
                            camera->getExisting_obj()[matched_idx].status = EXIT;
                        }
                    }else{
                        // not close to boundary, this could be marked as move
                        camera->getExisting_obj()[matched_idx].status = MOVE;
                    }
                }
                // TODO: judge the status in the cloud pls, combining the field view direction
                // update status: front mid rear, probably wouldn't matter at all
                // if (camera->getExisting_obj()[matched_idx].direction < 45 || ){

                // }

                // update existing bbox to be the new one
                camera->getExisting_obj()[matched_idx].box = camera->getNew_obj()[new_idx].box;
                // refresh cmt last image and bounding box as welll
//                    if (USE_CMT){
//                        Rect rect_tmp = Rect(camera->getExisting_obj()[matched_idx].box.left,camera->getExisting_obj()[matched_idx].box.top,camera->getExisting_obj()[matched_idx].box.right-camera->getExisting_obj()[matched_idx].box.left,camera->getExisting_obj()[matched_idx].box.bot-camera.camera->getExisting_obj()[matched_idx].box.top);
//                        //legalize the bbox
//                        rect_tmp = legalize_box(im0,rect_tmp);
//
//                        CMT cmt_tmp;
//                        cmt_tmp.initialize(im0_gray, rect_tmp, im0);
//                        cmt_vec.erase(cmt_vec.begin()+matched_idx);
//                        cmt_vec.insert(cmt_vec.begin()+matched_idx,cmt_tmp);
//                    }

                // TODO: add cld? the closest match is not robust when there are two cars moving towards each other.
            }else{
                // new instance
                // cout << "Object No. "  << "\n";
                Object_ID++;
                if (DEBUG) printf("New Object %d \n",Object_ID);
                // obj_box tmp_box = camera->getNew_obj()[new_idx].box;
                // obj_bb_box.push_back(tmp_box);
                Object tmp_obj;
                tmp_obj.box = camera->getNew_obj()[new_idx].box;
                tmp_obj.last_box = camera->getNew_obj()[new_idx].box;
                tmp_obj.id = Object_ID;
                tmp_obj.alive = true;
                tmp_obj.tracked = true;
                tmp_obj.evict = false;
                tmp_obj.status = ENTER;
                tmp_obj.displacement = 0;
                tmp_obj.direction = 0;
                tmp_obj.evict_count = 0;
                tmp_obj.move_count = 0;
                camera->pushBackExisting_obj(tmp_obj);
//                camera->getExisting_obj().push_back(tmp_obj);
                camera->getNew_obj()[new_idx].id = Object_ID;

//                    if (USE_CMT){
//                        //creating the rectangle for bb_box
//                        Rect rect_tmp = Rect(camera->getNew_obj()[new_idx].box.left,camera->getNew_obj()[new_idx].box.top,camera->getNew_obj()[new_idx].box.right-camera->getNew_obj()[new_idx].box.left,camera->getNew_obj()[new_idx].box.bot-camera->getNew_obj()[new_idx].box.top);
//                        //legalize the bbox
//                        rect_tmp = legalize_box(im0,rect_tmp);
//                        CMT cmt_tmp;
//                        cmt_tmp.initialize(im0_gray, rect_tmp, im0);
//                        cmt_vec.push_back(cmt_tmp);
////                            FILE_LOG(logINFO) << "Using " << rect_tmp.x << "," << rect_tmp.y << "," << rect_tmp.x + rect_tmp.width << "," << rect_tmp.y+rect_tmp.height
////                                              << "for objs #" << int(camera->getExisting_obj().size()) << " as initial bounding box.";
//
//                    }

            }
        }
    }
}

void MultiObjMatcher::newObjMotionFilter(){
    for (int new_idx = 0;(new_idx<int(camera->getNew_obj().size())&&(camera->getNew_obj()[new_idx].box.left!=YOLO_MARK));new_idx++){

        if (YOLO_DETECTION_OUTPUT){
            io->yolo_obj_file << camera->getCurFrameId() << ", "<< "-1" << ", " << camera->getNew_obj()[new_idx].box.left << ", "<< camera->getNew_obj()[new_idx].box.top << ", "
                             << camera->getNew_obj()[new_idx].box.right - camera->getNew_obj()[new_idx].box.left << ", " << camera->getNew_obj()[new_idx].box.bot - camera->getNew_obj()[new_idx].box.top
                             << ", -1, -1, -1, -1" << endl;
        }

        // phase 1:
        double min_centroid_dist = 1000;
        int matched_idx = -1;

        for (int exist_idx = 0;exist_idx<int(camera->getExisting_obj().size())&&(camera->getExisting_obj()[exist_idx].box.left!=YOLO_MARK);exist_idx++){
            // shouldn't check alive, because you need dead box to rule out new box as well

            double tmp_dist = get_centroid_distance(camera->getExisting_obj()[exist_idx].box,camera->getNew_obj()[new_idx].box);
            // printf("getting centroid distance from box, %d %d %d %d,  and box,%d %d %d %d\n centroid distance: %f\n", camera->getExisting_obj()[exist_idx].box.left,camera->getExisting_obj()[exist_idx].box.top,camera->getExisting_obj()[exist_idx].box.right,camera->getExisting_obj()[exist_idx].box.bot,camera->getNew_obj()[new_idx].box.left,camera->getNew_obj()[new_idx].box.top,camera->getNew_obj()[new_idx].box.right,camera->getNew_obj()[new_idx].box.bot,tmp_dist);
            double tmp_scale = get_scale_distance(camera->getExisting_obj()[exist_idx].box,camera->getNew_obj()[new_idx].box);
            if ( tmp_dist < min_centroid_dist && same_box_test(tmp_dist,tmp_scale,STOP_CENTROID_THRESH, STOP_SCALE_THRESH)){
                min_centroid_dist = tmp_dist;
                matched_idx = exist_idx;
            }

        }
        if (matched_idx!=-1){
            // mark don't care, not moving
            if (DEBUG) printf("Object %d is seen not moving\n", camera->getExisting_obj()[matched_idx].id);
            camera->getExisting_obj()[matched_idx].alive = false;
            camera->getExisting_obj()[matched_idx].evict = false;
            camera->getExisting_obj()[matched_idx].evict_count = 0;
            // mark them matched by giving -1 id
            camera->getNew_obj()[new_idx].id = -1;
            // NO YOU CAN NOT delete cmt vector, CMT VECTOR INDEX MUST MATCH OBJ_BB_BOX INDEX
            continue;
        }
        // printf("Object %d might moved\n", camera->getExisting_obj()[matched_idx].id);
    }
}

double MultiObjMatcher::get_centroid_distance(struct obj_box box1, struct obj_box box2){
    int centroid1_x =(box1.left+box1.right) / 2;
    int centroid1_y = (box1.top+box1.bot) / 2;
    int centroid2_x = (box2.left+box2.right) / 2;
    int centroid2_y = (box2.top+box2.bot) / 2;
    // double centroid_dist = sqrt((centroid1_x-centroid2_x)*(centroid1_x-centroid2_x) + (centroid1_y-centroid2_y)*(centroid1_y-centroid2_y));
    double centroid_dist = get_dist(centroid1_x,  centroid1_y,  centroid2_x,  centroid2_y);
    // printf("getting centroid distance from box, %d %d %d %d,  and box,%d %d %d %d\n centroid distance: %f\n", box1.left,box1.top,box1.right,box1.bot,box2.left,box2.top,box2.right,box2.bot,centroid_dist);
    return centroid_dist;
}


double MultiObjMatcher::get_centroid_direction(struct obj_box new_box, struct obj_box old_box){
    int exist_centroid_x =(old_box.left+old_box.right) / 2;
    int exist_centroid_y = (old_box.top+old_box.bot) / 2;
    int new_centroid_x = (new_box.left+new_box.right) / 2;
    int new_centroid_y = (new_box.top+new_box.bot) / 2;
    // int new_centroid_x = (old_box.left+camera->getNew_obj()[new_idx].box.right)/2;
    // int new_centroid_y = (camera->getNew_obj()[new_idx].box.top+camera->getNew_obj()[new_idx].box.bot)/2;
    // int exist_centroid_x = (camera->getExisting_obj()[matched_idx].box.left+camera->getExisting_obj()[matched_idx].box.right)/2;
    // int exist_centroid_y = (camera->getExisting_obj()[matched_idx].box.top+camera->getExisting_obj()[matched_idx].box.bot)/2;
    return atan2(new_centroid_y-exist_centroid_y,new_centroid_x - exist_centroid_x) *180 / PI;
    // return -pi to pi
    //         90
    // -/+180--------0
    //        -90
}

double MultiObjMatcher::get_scale_distance(struct obj_box box1, struct obj_box box2){
    double scale_dist = double((box1.right-box1.left)*(box1.top-box1.bot)) / double((box2.right-box2.left)*(box2.top-box2.bot));
    return scale_dist;
}
bool MultiObjMatcher::same_box_test(double centroid_dist, double scale_dist, double centroid_thresh, double scale_thresh){
    if (scale_dist > scale_thresh && scale_dist < 1/scale_thresh && (centroid_dist < centroid_thresh || centroid_thresh == -1 )) {
        return true;
    }else{
        if (DEBUG) printf("centroid distance/thresh: %f/%f, scale gap/thresh: %f/%f\n", centroid_dist,centroid_thresh,scale_dist,scale_thresh);
        return false;
    }
}

// centroid thresh in pixels
// scale thresh < 1
bool MultiObjMatcher::same_box_test_from_box(struct obj_box box1, struct obj_box box2, double centroid_thresh, double scale_thresh){
    double centroid_dist = get_centroid_distance(box1, box2);
    double scale_dist = get_scale_distance(box1, box2);
    return same_box_test(centroid_dist, scale_dist, centroid_thresh, scale_thresh);
}



bool MultiObjMatcher::check_boundary(int cols, int rows, obj_box box){

    if (box.right > cols-BOUNDARY_THRESH || box.left < BOUNDARY_THRESH || box.top <BOUNDARY_THRESH || box.bot > rows-BOUNDARY_THRESH){
        return false;
    }
    return true;
}

bool MultiObjMatcher::check_movement(Point2f mov_vec, Point2f frame_total_op_flow){

    if (MOBILE_CAM){
        double dir_mov_vec = atan2(mov_vec.y, mov_vec.x) *180 / PI;
        double dir_frame_total_op_flow = atan2(frame_total_op_flow.y, frame_total_op_flow.x) *180 / PI;
        if (DEBUG) printf("object mv: %f, dir: %f, backgroud mv: %f, dir: %f\n", norm(mov_vec), dir_mov_vec, norm(frame_total_op_flow), dir_frame_total_op_flow);
        // if (is_same_dir(dir_mov_vec,dir_frame_total_op_flow)){
        //     return abs(norm(mov_vec) - norm(frameTotalOpticalFlow)) > STOP_CENTROID_THRESH
        // }else{
        //     return norm(mov_vec) > STOP_CENTROID_THRESH;
        // }
        return norm(mov_vec - frame_total_op_flow) > STOP_CENTROID_THRESH;
    }else{
        return norm(mov_vec) > STOP_CENTROID_THRESH;
    }
}

MultiObjMatcher::MultiObjMatcher(MultiObjTracker *multiObjTracker, Cam *camera, IO *io) : multiObjTracker(
        multiObjTracker), camera(camera), io(io) {
    Object_ID = 0;
}

MultiObjMatcher::~MultiObjMatcher() {

}

