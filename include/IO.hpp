//
// Created by nsl on 9/20/16.
//

#ifndef PROJECT_IO_H
#define PROJECT_IO_H

#include "globals.hpp"
#include "AugmentedVR.hpp"
#include <fstream>
using namespace std;

class AugmentedVR;
class IO {
public:
//    IO();

    IO(AugmentedVR *myAVR);

    virtual ~IO();
//    void init(const string&depthFP, const string& traceLogFP);
private:
    ofstream depthFile;
    ofstream TimeFile;

    ofstream logFile;
//    fstream traceLogFile;
    AugmentedVR* myAVR;


public:
    void writeCurrentStereoFrame();
    void logCurrentFrame();
};


#endif //PROJECT_IO_H
