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

#include <sstream>
#include <string>
#include <fstream>

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

enum ROLE{
    SOLO, SENDER, RECEIVER, TRANSCEIVER
};


void loadParams(){

    ifstream config("config.txt");
    string toggleName;
    string toggleValue;
    while(std::getline( config,toggleName)){
        std::getline(config,toggleValue);
        if (toggleName.compare("DEBUG")==0) DEBUG = stoi(toggleValue);
        if (toggleName.compare("VISUAL")==0) VISUAL = toggleValue.compare("true")==0;
        if (toggleName.compare("COOP")==0) COOP = toggleValue.compare("true")==0;
        if (toggleName.compare("Parallel_TXRX")==0) Parallel_TXRX = toggleValue.compare("true")==0;
        if (toggleName.compare("ADAPTIVE_STREAMING")==0) ADAPTIVE_STREAMING = toggleValue.compare("true")==0;
        if (toggleName.compare("VehicleControl")==0) VehicleControl = toggleValue.compare("true")==0;
        if (toggleName.compare("DYNAMICS")==0) DYNAMICS = toggleValue.compare("true")==0;
        if (toggleName.compare("PAUSE_FLAG")==0) PAUSE_FLAG = toggleValue.compare("true")==0;
        if (toggleName.compare("ZEDCACHESIZE")==0) ZEDCACHESIZE = stoi(toggleValue);
        if (toggleName.compare("MAX_COUNT")==0) MAX_COUNT = stoi(toggleValue);
        if (toggleName.compare("V2VDEBUG")==0) V2VDEBUG = toggleValue.compare("true")==0;
        if (toggleName.compare("PIPELINE")==0) PIPELINE = toggleValue.compare("true")==0;
        if (toggleName.compare("SHOW_CAMMOTION")==0) SHOW_CAMMOTION = toggleValue.compare("true")==0;
        if (toggleName.compare("TXFRAME_FOREVAL")==0) TXFRAME_FOREVAL = toggleValue.compare("true")==0;
        if (toggleName.compare("LOCKDEBUG")==0) LOCKDEBUG = toggleValue.compare("true")==0;
        if (toggleName.compare("SILENCENOMOTION")==0) SILENCENOMOTION= toggleValue.compare("true")==0;

    }

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

    argidx++;
    if (argc > 2) {

        string identifier = argv[argidx++];
        if (identifier == "left"){
            COOP = true;
        }
        cout << "Identifier: " << identifier << endl;
        int role = stoi(argv[argidx++]);
        switch (role){
            case SOLO: RX = false;TX = false;SEND = false;CamId = 0;RxCamId = 0;cout << "Role: SOLO Mode"<<endl;break;
            case SENDER: RX = false;TX = true;SEND = true;CamId = 0;RxCamId = 1;cout << "Role: SENDER Mode"<<endl;break;
            case RECEIVER: RX = true;TX = false;SEND = false;CamId = 1;RxCamId = 0;cout << "Role: RECEIVER Mode"<<endl;break;
            case TRANSCEIVER: RX = true;TX = true;SEND = true;CamId = 0;RxCamId = 1;cout << "Role: RECEIVER Mode"<<endl;break;
        }

    }


    if (argc > 4){
        startFrameId = stoi(argv[argidx++]);
        lengthInFrame = stoi(argv[argidx++]);
        cout << "Starting frame: " << startFrameId << endl;
        cout << "Length: " << lengthInFrame<< endl;
    }

    if (argc > 6){
        ReuseMap = true;
        mapFile = argv[argidx++];
        cout << "Map File: " << mapFile << endl;
    }

    loadParams();

    mV = new VCluster(mapFile, VPath[0]);

#ifdef EVAL
    gettimeofday(&tInit, NULL);
#endif

    if (VISUAL&&PCVISUAL){
        mvThread = new std::thread(run);
        glutCloseFunc(close);
        glutMainLoop();
    } else{
        mV->run();
    }
    return 0;
}


