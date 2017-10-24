///////////////////////////////////////////////////////////////////////////
///////////////////////// CROWDSOURCED HD MAP /////////////////////////////
//////////////////////////////////////////////////////////////////////////

////////////// MODES ////////////////
///////////// 1 for ON //////////////
//////////// 0 for OFF /////////////


/********************************************************************************
 ** This sample demonstrates how to grab images and change the camera settings **
 ** with the ZED SDK                                                           **
 ********************************************************************************/


//// Standard includes
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
//// ZED include
#include <sl/Camera.hpp>
#include "../src/ORB_SLAM2/include/System.h"
//// Library Files
#include <ZEDInitialization.h>

//Open CV files

#define ON 1
#define OFF 0

#define DEBUG_MODE ON
#define GET_STATIC_FEATURES OFF
#define SPATIAL_FEATURE_MODE OFF
#define ORB_SLAM_MODE ON
#define ZED_SLAM_MODE OFF


/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// PATH VARIABLES ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
const std::string pathToProject = "/home/fawad/Crowdsourced_HD_Map/";
const std::string pathToAuxillaryData = "/media/fawad/DATA/Crowdsourced_HD_Map_Aux_Files/";
const std::string pathToMapFiles = pathToAuxillaryData + "Map_Files/";
/////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// ORB SLAM INPUT FILES ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// VOCABULARY FILE //////////////////////////////////////
const std::string pathToVocabFile = pathToProject + "ORB_SLAM2/Vocabulary/ORBvoc.bin";
//////////////////////////////// CALIBRATION FILE //////////////////////////////////////
const std::string pathToCalibFile = pathToProject + "CamCalib.yaml";


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// INPUTS ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////// INPUT SVO FILE ////////////////////////////////////////////////
const sl::String pathToInputFile = "/media/fawad/DATA/"
        "ZED_SVO_Files/OutdoorFile.svo";
        //"Home_Block_Scan/Reference_Scans/Reference_Scan_One/Multi_Session/"
        //"SessionOne.svo";
        //"Crowdsourced_HD_Map_Aux_Files/ZED_SVO_Files/"
        //"Home_Block_Scan/Reference_Scans/Reference_Scan_One"
        //"/Multi_Session/Stretch90m.svo";
        //"Reference_Scan_One/Multi_Session/SessionOne.svo";

///media/fawad/DATA/Crowdsourced_HD_Map_Aux_Files/ZED_SVO_Files/Home_Block_Scan/Reference_Scans/Reference_Scan_One/Multi_Session

//////////////////////////////// LOAD MAP FILE ////////////////////////////////////////////
const std::string pathToLoadMapFile = pathToMapFiles +
        "LongerDistanceExperiments/SLAM_Map_File";//do not include .bin


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// OUTPUTS ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// SAVE MAP FILE ///////////////////////////////////////////////
const std::string pathToSaveMapFile = pathToMapFiles +
        "LongerDistanceExperiments/SLAM_Map_File";

///////////////////////////// SAVE TRAJECTORY FILE ///////////////////////////////////////
const std::string pathToSaveTrajectory = pathToAuxillaryData + "Results/LongerDistanceExperiments/Localization_One";

/////////////////////////////////////////////////////////////////////////////

//// Using std and sl namespaces
using namespace std;
using namespace sl;



//constants
const bool IS_LIVE_MODE = false;
const bool IS_CAMERA_FLIPPED = true;

//ORB_SLAM2 Variables
const bool ORBVisualization = true;
const bool saveMap = false;
const bool reuseMap = true;
const bool runLocalizationMode = true;
const bool saveTrajectory = false;

Camera *ptrToCamObj;//pointer to ZED Camera Object


int main(int argc, char **argv) {

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////// INITIALIZATION ////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Welcome to Crowdsourced HD Map" << endl;
    ///////// Create a ZED camera //////////////////////////
    ///////// Initialize and open the camera ///////////////
    Camera zedCamObject;//camera object
    InitParameters initParameters;//initial parameters
    if (IS_CAMERA_FLIPPED)
        initParameters.camera_image_flip = true;
    if (initializeZED(&zedCamObject, &initParameters, IS_LIVE_MODE, pathToInputFile) != SUCCESS)//initialize the camera
    {
        cout << "Shutting down application" << endl;
        return -1;
    }


    ORB_SLAM2::System *ptrToORBSLAMSystem;

#if ORB_SLAM_MODE
    ptrToORBSLAMSystem = initializeORBSlam(pathToVocabFile, pathToCalibFile, ORBVisualization, saveMap, reuseMap, pathToLoadMapFile + ".bin", runLocalizationMode, saveTrajectory, pathToSaveTrajectory);
#endif


    //Runtime parameters
    RuntimeParameters runtimeParams;
    runtimeParams.sensing_mode = SENSING_MODE_STANDARD;
    runtimeParams.enable_depth = true;
    runtimeParams.enable_point_cloud = true;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// MAIN PROGRAM LOOP //////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    cout << "Number of frames = " << zedCamObject.getSVONumberOfFrames() << endl;


    for (int frameCounter = 0;frameCounter < zedCamObject.getSVONumberOfFrames(); frameCounter++)
    {
        //cout << frameCounter << "/" << zedCamObject.getSVONumberOfFrames() << endl;



        if (zedCamObject.grab(runtimeParams) == SUCCESS) {
            sl::Mat leftView, rightView;
            //Left and right images
            ERROR_CODE errOne = zedCamObject.retrieveImage(leftView, VIEW_LEFT);
            ERROR_CODE errTwo = zedCamObject.retrieveImage(rightView, VIEW_RIGHT);
            //ORB_SLAM2 Code
            //Feed slam the next frame

#if ORB_SLAM_MODE
            //Run ORB_SLAM2
            runORBSLAMTracking(ptrToORBSLAMSystem, leftView, rightView, frameCounter);

#endif


        }

    }



#if ORB_SLAM_MODE
        cout << "Shutting down ORB SLAM2" << endl;
    shutDownORBSLAM(ptrToORBSLAMSystem, "", true, pathToSaveMapFile);
    //if (runLocalizationMode == false)
        saveTrajectoryORBSLAMinSLAMMode (ptrToORBSLAMSystem, pathToSaveTrajectory+"_ORBSLAM2.txt");
#endif

#if ZED_SLAM_MODE
    //shutDownZEDSLAM(&zedCamObject, false);
#endif



    zedCamObject.close();
    cout << "Closing down ...." << endl;
    //cout << "Press any key to exit ....." << endl;
    //cin.get();
    return EXIT_SUCCESS;
}

