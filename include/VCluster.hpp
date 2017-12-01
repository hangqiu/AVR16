//
// Created by hang on 3/2/17.
//

#ifndef PROJECT_VCLUSTER_H
#define PROJECT_VCLUSTER_H


#include "AugmentedVR.hpp"
#include "Displayer.hpp"
#include "ObjReceiver.hpp"
#include "ObjSender.hpp"
#include "globals.hpp"
#include "pcCodec.hpp"
#include "PathPlanner.hpp"

class VCluster {
private:
    PathPlanner* mPathPlanner;
    Displayer* mDisplayer;
    ObjSender* mSender;
    ObjReceiver* mReceiver;
    pcCodec* mCodec;
    int frameSeqRx;
    unsigned long long int timeRx;

    std::thread* displayThread;
    bool live = false;

public://public variables


    AugmentedVR** VNode;

public:
    VCluster();
    VCluster(const string mapFile, string VPath);
//    VCluster(bool live, const string mapFile, int argc, char** argv, string VPath);
    ~VCluster();
    void exit();

    void run();

    void postProcess();

    void compressDynamic();
    void visualize();
    void TXRX();
    void TXRX_viaDisk();

    bool PreProcess();

    void segmentation();

    void RoadDetection();

    void initDisplay();
};


#endif //PROJECT_VCLUSTER_H
