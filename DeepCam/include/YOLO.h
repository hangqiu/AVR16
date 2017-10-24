//
// Created by nsl on 9/16/16.
//

#ifndef MOBILEVIDEONET_CLEAN_YOLO_H
#define MOBILEVIDEONET_CLEAN_YOLO_H

#include <iostream>
#include <stdlib.h>
#include <vector>



#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "Object.h"
#include "globals.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "DeepCam/YOLO/detection.h"

#ifdef __cplusplus
}
#endif

class YOLO {

private:
    int curObjId;
    bool compareByScore(const struct Object &a, const struct Object &b){
        return a.box.score > b.box.score;
    }
public:
    network net;

public:
    YOLO();

    virtual ~YOLO();

    void init(string cfgfile, string weightfile);
    std::vector<Object> run_yolo(cv::Mat im0, char* camID);
};


#endif //MOBILEVIDEONET_CLEAN_YOLO_H
