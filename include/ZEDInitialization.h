//
// Created by fawad on 9/6/17.
//

#ifndef CROWDSOURCED_HD_MAP_ZEDINITIALIZATION_H
#define CROWDSOURCED_HD_MAP_ZEDINITIALIZATION_H

#include <sl/Camera.hpp>
#include "../src/ORB_SLAM2/include/System.h"

std::string trajectoryFilePath;
ofstream trajectoryFile;
bool isTracking;
bool isSaveTrajectory;


using namespace sl;

//Initialize ZED Camera
int initializeZED (Camera *, InitParameters *, bool, sl::String);
//Initialize ZED SLAM
void initializeZEDSLAM (Camera *, TrackingParameters *, String, bool , String = NULL);

//SL Mat to CV Mat conversion
cv::Mat slMat2cvMat2(sl::Mat&);


//ORB SLAM2 Initialization
ORB_SLAM2::System *initializeORBSlam (string vocabFile, string settingFile, bool visualization, bool saveMap, bool reuseMap, string pathToLoadMapFile, bool runLocalizationMode, bool saveTrajectory, string pathToTrajectoryFile);
void runORBSLAMTracking (ORB_SLAM2::System *, sl::Mat, sl::Mat, int);//periodic tracking using ORB SLAM

void saveTrajectoryORBSLAMinSLAMMode (ORB_SLAM2::System * ptrToORBSLAM, const std::string pathToSaveTrajectory);


//ERROR_CODE enableSpatialMap (Camera *, SpatialMappingParameters *);
void initializeORBSlam (string, string, bool);//initialize ORB SLAM system

void shutDownORBSLAM (ORB_SLAM2::System *, std::string, bool, std::string);//Shut down ORB SLAM and record trajectory
void runZEDSLAM (Camera *);
void shutDownZEDSLAM (Camera *, bool, String = NULL);




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// ZED CAMERA FUNCTIONS ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ZED Camera Initialization
int initializeZED (Camera *ptrToZEDCamera, InitParameters *initParameters, bool liveMode = false, sl::String pathToInputFile = "")
{
    //InitParameters for Initializing ZED Camera
    initParameters->camera_resolution = RESOLUTION_HD720;
    initParameters->camera_fps = 30;
    initParameters->depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters->coordinate_units = UNIT_METER;
    initParameters->coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;


    if (liveMode == false)
        initParameters->svo_input_filename = pathToInputFile;

    ERROR_CODE zedCamStatus = ptrToZEDCamera->open(*initParameters);

    if (zedCamStatus != SUCCESS) {
        cout << "Error in opening ZED Camera " << errorCode2str(zedCamStatus) << endl;
        ptrToZEDCamera->close();
        return -1;
    }
    else
        cout << "Initialized ZED SDK" << endl;

    return SUCCESS;
}

void saveTrajectoryORBSLAMinSLAMMode (ORB_SLAM2::System * ptrToORBSLAM, const std::string pathToSaveTrajectory)
{
    ptrToORBSLAM->SaveTrajectoryKITTI(pathToSaveTrajectory);
    cout << "Trajectory saved at " << pathToSaveTrajectory << endl;
}

//ZED SLAM initialization
void initializeZEDSLAM (Camera *ptrToZEDCamera, TrackingParameters *ptrToTrackingParams, const string pathToZEDTrackingOutput, bool reuseMap = false, String pathToMapFile = "")
{
    ptrToTrackingParams->initial_world_transform = sl::Transform::identity();
    ptrToTrackingParams->enable_spatial_memory = true;
    if (reuseMap == true)
        ptrToTrackingParams->area_file_path = pathToMapFile;//give path to map file
    //zedTrackingFile.open (pathToZEDTrackingOutput);


    ERROR_CODE trackingErrorStatus = ptrToZEDCamera->enableTracking(*ptrToTrackingParams);

    if (trackingErrorStatus != SUCCESS)
    {
        std::cout << "Error in initializing ZED SLAM " << errorCode2str(trackingErrorStatus) << endl;
    }
    else
        std::cout << "Initialized ZED SLAM" << endl;

    return;
}


//SL Mat to CV Mat conversion
cv::Mat slMat2cvMat2(sl::Mat& input) {
    //convert MAT_TYPE to CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE_32F_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE_32F_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE_32F_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE_8U_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE_8U_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE_8U_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE_8U_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }

    // cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    //cv::Mat and sl::Mat will share the same memory pointer
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// ORB SLAM 2 Interface ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ORB SLAM2 Functions
ORB_SLAM2::System *initializeORBSlam (string vocabFile, string settingFile, bool visualization, bool saveMap, bool reuseMap, string pathToLoadMapFile, bool runLocalizationMode, bool saveTrajectory, string pathToSaveTrajectory)
{

    isSaveTrajectory = saveTrajectory;
    if (runLocalizationMode == true || saveTrajectory == true)
        isTracking = true;
    else
        isTracking = false;


    if (isSaveTrajectory == true)
    {
        trajectoryFilePath = pathToSaveTrajectory + ".txt";
        trajectoryFile.open(trajectoryFilePath);
    }
    else;


    return new ORB_SLAM2::System (vocabFile, settingFile, ORB_SLAM2::System::STEREO, visualization, saveMap, reuseMap, pathToLoadMapFile, runLocalizationMode);
}

void runORBSLAMTracking (ORB_SLAM2::System *ptrToORBSLAMSystem, sl::Mat zedLeftView, sl::Mat zedRightView, int timeCounter)
{
    cv::Mat leftViewCV, rightViewCV, leftViewCVGray, rightViewCVGray;
    leftViewCV = slMat2cvMat2(zedLeftView);
    rightViewCV = slMat2cvMat2(zedRightView);

    //Left and right gray scale images
    cv::cvtColor(leftViewCV, leftViewCVGray, CV_BGR2GRAY);
    cv::cvtColor(rightViewCV, rightViewCVGray, CV_BGR2GRAY);

    //cv::Mat tcw = ptrToORBSLAMSystem->TrackMonocular(leftViewCV, timeCounter);


    cv::Mat tcw = ptrToORBSLAMSystem->TrackStereo(leftViewCV, rightViewCV, timeCounter);//

//    cout << "Crashed here" << endl;
    if (tcw.rows == 4 && tcw.cols == 4)
    if (isSaveTrajectory == true) {
        //trajectoryFile << tcw << endl;
        Eigen::MatrixXd A(4, 4);
        A << tcw.at<float>(0, 0), tcw.at<float>(0, 1), tcw.at<float>(0, 2), tcw.at<float>(0, 3),
                tcw.at<float>(1, 0), tcw.at<float>(1, 1), tcw.at<float>(1, 2), tcw.at<float>(1, 3),
                tcw.at<float>(2, 0), tcw.at<float>(2, 1), tcw.at<float>(2, 2), tcw.at<float>(2, 3),
                0, 0, 0, 1;

        A = A.inverse();

        {
            trajectoryFile << A(0, 3) << "," << A(1, 3) << "," << A(2, 3) << endl;
            //tcw.at<float>(0,3) << "," << tcw.at<float>(1,3) << "," << tcw.at<float>(2,3) << endl;
            /*
            trajectoryFile << tcw.at<float>(0, 0) << "," << tcw.at<float>(0, 1) << "," << tcw.at<float>(0, 2) << ","
                           << tcw.at<float>(0,3)
                           << "," <<
                           tcw.at<float>(1, 0) << "," << tcw.at<float>(1, 1) << "," << tcw.at<float>(1, 2) << ","
                           << tcw.at<float>(1,3)
                           << "," <<
                           tcw.at<float>(2, 0) << "," << tcw.at<float>(2, 1) << "," << tcw.at<float>(2, 2) << ","
                           << tcw.at<float>(2,3)
                           << endl;
                           */
      }
 }


//    } else
            //      cout << "Missed" << endl;
//    cout << "Hello" << endl;
            return;
//}
        }

void shutDownORBSLAM (ORB_SLAM2::System *ptrToORBSLAMSystem, string filePathToSaveTrajectoryInfo, bool updateMapFile, string pathToSaveMapFile)
{
    if (isTracking == true)
        trajectoryFile.close();
    ptrToORBSLAMSystem->Shutdown(pathToSaveMapFile);

    return;
}

#endif //HD_MAP_PROJECT_ZEDINITIALIZATION_H
