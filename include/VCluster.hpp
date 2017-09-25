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

class VCluster {
private:

    Displayer* mDisplayer;
    ObjSender* mSender;
    ObjReceiver* mReceiver;
    pcCodec* mCodec;
    int frameSeqRx;
    long timeRx;

    std::thread* displayThread;

public://public variables
    AugmentedVR** VNode;

public:
    VCluster();
    VCluster(bool live, const string mapFile, int argc, char** argv, string VPath);
    ~VCluster();
    void exit();

    void run();

    void postProcess();

    void compressDynamic();
    void visualize();
    void TXRX();

    void PreProcess();

    void segmentation();

    void initDisplay();
};


#endif //PROJECT_VCLUSTER_H
