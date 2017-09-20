//
// Created by nsl on 9/17/16.
//

#ifndef MOBILEVIDEONET_CLEAN_IO_H
#define MOBILEVIDEONET_CLEAN_IO_H


#include <obj_record.pb.h>
#include "globals.h"
#include "Object.h"
#include "DeepCam.h"

class Cam;

class IO {
private:
//    Mat im_0, im_0_gray; // original frame
//    Mat curIm, curGray; // resized frame
////    Mat im, im_gray;
//    Mat prevIm, prevGray;

//    vector<Object> existing_obj;
//    vector<Object> moved_obj;
//    vector<Object> new_obj;

    Cam* camera;
    tracklets::TrackletBook obj_book;
    char outputPrefix[50];

public:
    ifstream metadata_file;
    ofstream obj_file;
    ofstream opflow_file;
    ofstream yolo_obj_file;
    ofstream latency_file;
    ofstream total_latency_file;

public:
    IO(char* DatasetPath, char* CamID, char* metadata_path);

    virtual ~IO();

    void writeOriginalFrame(char* filepath);
    void writeResizedFrame(char* filepath);

    void printExistingObj();

    void logObjects();

    void writeProtobuf(char *proto_path);

    void initProtobuf(char *CamID, double lat, double lon, char *input_path);

    void writeCLD(Mat img, char *filepath);

    void setCamMeta(Cam* myCam);
};


#endif //MOBILEVIDEONET_CLEAN_IO_H
