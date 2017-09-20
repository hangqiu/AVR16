//
// Created by nsl on 9/16/16.
//

#ifndef MOBILEVIDEONET_CLEAN_OBJECT_H
#define MOBILEVIDEONET_CLEAN_OBJECT_H

#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif

#include "DeepCam/YOLO/detection.h"

#ifdef __cplusplus
}
#include <string.h>

#endif

//using namespace std;

class Object{

public: // TODO make it private
    int id;
    obj_box box;
    std::string status;//enter/mid/exit
    bool alive;// true means moving, false means stop
    bool tracked; // true means it's tracked, false meanse no matching new boxes.
    bool evict;// true evict cound should increase by frame
    double direction;//0-360
    double displacement; // >0 moving vector magnitude
    std::string view;//front/mid/back
    int evict_count; //
    int move_count;// how how many frames is it moving; output if > 50% of frames during a cycle is moving
    obj_box last_box;
    // Mat LastFrame;
    // int LastFrameID;
    // vector<Point2f> LastKeyPoints;
    // LKT_IDX kpt_idx; // indexing the keypoints that belong to this box, from last frame
public:
    void setId(int id);

    void setBox(const obj_box &box);

    void setStatus(const std::string &status);

    void setAlive(bool alive);

    void setTracked(bool tracked);

    void setEvict(bool evict);

    void setDirection(double direction);

    void setDisplacement(double displacement);

    void setView(const std::string &view);

    void setEvict_count(int evict_count);

    void setMove_count(int move_count);

    void setLast_box(const obj_box &last_box);
};

#endif //MOBILEVIDEONET_CLEAN_OBJECT_H
