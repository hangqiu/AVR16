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
#include <sys/time.h>
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
//#include "opencv2/xfeatures2d.hpp"

//ZED Includes
//#include <zed/Mat.hpp>
//#include <zed/Camera.hpp>
//#include <zed/utils/GlobalDefine.hpp>

#include <sl/Camera.hpp>

#include <landmark.hpp>
#include <landmark_matchinfo.hpp>
#include <include/Displayer.hpp>
#include <include/globals.hpp>
#include <include/ObjSender.hpp>
#include <include/ObjReceiver.hpp>
#include <include/VCluster.hpp>


//our point cloud generator and viewer.
//#include "Viewer.hpp"
//#include "PointCloud.hpp"

//#include "../ORB_SLAM2/include/System.h"
#include "System.h"

#include "AugmentedVR.hpp"


//#define EVAL;

using namespace sl;
using namespace std;
using namespace cv;
//using namespace cv::xfeatures2d;




VCluster* mV;
std::thread* mvThread;

void run(){

    mV->run();
}


void close() {
    quit = true;
    mvThread->join();
    mV->exit();
    cout << "System shuts down" << endl;
}
///////////////////////////////////////////////////////////main  function////////////////////////////////////////////////////
int main(int argc, char **argv) {

#ifdef WIN32
    std::cout << "Multiple ZEDs are not available under Windows" << std::endl;
    return -1;
#endif
// parse input
//    if (argc != 4) cout << "vidoe 1, video 2, map file\n";
    if (argc <1 ) cout << "vidoe path, id, map file\n";
    string VPath[NUM_CAMERAS];
    int argidx =0;
    for (argidx = 0;argidx < NUM_CAMERAS; argidx++) {
        VPath[argidx] = argv[argidx+1];
        cout << "Video " << argidx << ": " << VPath[argidx] << endl;
    }


//    const string mapFile = argv[argidx+1];
//    cout << "Map File: " << mapFile << endl;

    string mapFile;

    string identifier = argv[argidx+1];
    if (identifier == "left"){
        COOP = true;
    }

    if (argc > 3){
        startFrameId = stoi(argv[argidx+2]);
        lengthInFrame = stoi(argv[argidx+3]);
    }

    if (argc > 5){
        ReuseMap = true;
        mapFile = argv[argidx+4];
        cout << "Map File: " << mapFile << endl;
    }

    mV = new VCluster(false, mapFile, argc, argv, VPath[0]);

#ifdef EVAL
    gettimeofday(&tInit, NULL);
#endif

    if (VISUAL){
        mvThread = new std::thread(run);
        glutCloseFunc(close);
        glutMainLoop();
    } else{
        mV->run();
    }
    return 0;
}


