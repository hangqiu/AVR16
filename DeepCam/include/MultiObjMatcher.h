//
// Created by nsl on 9/17/16.
//

#ifndef MOBILEVIDEONET_CLEAN_MULTIOBJMATCHER_H
#define MOBILEVIDEONET_CLEAN_MULTIOBJMATCHER_H

#include "globals.h"
#include "Object.h"
#include "MultiObjTracker.h"
#include "DeepCam.h"


class Cam;
class IO;
class MultiObjTracker;

class MultiObjMatcher {
private:

//    vector<Object> existing_obj;
//    vector<Object> moved_obj;
//    vector<Object> new_obj;


    MultiObjTracker* multiObjTracker;
    Cam* camera;
    IO* io;

    int Object_ID;

public:

    MultiObjMatcher(MultiObjTracker *multiObjTracker, Cam *camera, IO *io);

    virtual ~MultiObjMatcher();

    void updateStaleness();

    void matchingMovedObj();

    void newObjMotionFilter();

    double get_centroid_distance(obj_box box1, obj_box box2);

    double get_centroid_direction(obj_box new_box, obj_box old_box);

    double get_scale_distance(obj_box box1, obj_box box2);

    bool same_box_test(double centroid_dist, double scale_dist, double centroid_thresh, double scale_thresh);

    bool same_box_test_from_box(obj_box box1, obj_box box2, double centroid_thresh, double scale_thresh);

    Rect legalize_box(Mat im0, Rect bbox);

    bool check_boundary(int cols, int rows, obj_box box);

    bool check_movement(Point2f mov_vec, Point2f frame_total_op_flow);
};


#endif //MOBILEVIDEONET_CLEAN_MULTIOBJMATCHER_H
