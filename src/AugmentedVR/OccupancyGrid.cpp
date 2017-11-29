//
// Created by hang on 11/26/17.
//

#include <queue>
#include "OccupancyGrid.hpp"


OccupancyGrid::OccupancyGrid(double gridWidth,double gridHeight,double XMin,double XMax,double ZMin,double ZMax)
        :gridWidth(gridWidth),gridHeight(gridHeight),XMin(XMin),XMax(XMax),ZMin(ZMin),ZMax(ZMax)
{
    XNum = int((XMax-XMin) / gridWidth);
    ZNum = int((ZMax-ZMin) / gridHeight);

    OccupancyStatus.resize(XNum);

    for (int i=0;i<XNum;i++){
        OccupancyStatus[i].resize(ZNum);
        for (int j=0;j<ZNum;j++){
            OccupancyStatus[i][j]=0;
        }
    }
    roadOutputFile = ofstream("Road.txt");

}

OccupancyGrid::~OccupancyGrid(){
    roadOutputFile.close();
}

void OccupancyGrid::ClearOccupancyStatus(){
    for (int i=0;i<XNum;i++){
        for (int j=0;j<ZNum;j++){
            OccupancyStatus[i][j]=0;
        }
    }
}

void OccupancyGrid::ConvertPCAndSetOccupancyGrid_ManualPlaneModel(int frameSeq, cv::Mat &pc, sl::Mat &slpc, float A, float B, float C,
                                                                  float D){
    ///reset to 0
    ClearOccupancyStatus();

    cvMat2slMat(pc, slpc,sl::MEM_CPU);
    float * p_cloud = slpc.getPtr<float>(sl::MEM_CPU); // Get the pointer
    for (int i=0;i<pc.cols;i++){
        for (int j=0;j<pc.rows;j++){
            int index = (j*pc.cols+i)*4;
            float x = p_cloud[index];
            float y = p_cloud[index+1];
            float z = p_cloud[index+2];
            float rgba = p_cloud[index+3];
            if (!isValidMeasure(x)) continue;


            int XIdx = int((x-XMin) / gridWidth);
            int ZIdx = int((z-ZMin) / gridHeight);
            if (XIdx<0 || XIdx >= XNum || ZIdx < 0 || ZIdx >= ZNum) continue;

            if (A*x+B*y+C*z+D<0){
                /// road
                OccupancyStatus[XIdx][ZIdx] ++;
            }else{
                /// occupied
                OccupancyStatus[XIdx][ZIdx] --;
            }
        }
    }

    /// color road plane as occupancy grid
    int pointNum = 0;
    double detectedRoadArea = 0;
    for (int i=0;i<pc.cols;i++){
        bool gridIsRoad = false;
        for (int j=0;j<pc.rows;j++){
            int index = (j*pc.cols+i)*4;
            float x = p_cloud[index];
            if (!isValidMeasure(x)) continue;
            float y = p_cloud[index+1];
            float z = p_cloud[index+2];

            int XIdx = int((x-XMin) / gridWidth);
            int ZIdx = int((z-ZMin) / gridHeight);
            if (XIdx<0 || XIdx >= XNum || ZIdx < 0 || ZIdx >= ZNum) continue;

            if (A*x+B*y+C*z+D<0 && OccupancyStatus[XIdx][ZIdx]>0){
                /// road
                p_cloud[index+3] = 0xFFFFFFFF;
                pointNum ++;
                gridIsRoad = true;
            }
        }
    }
    for (int i=0;i<XNum;i++) {
        for (int j = 0; j < ZNum; j++) {
            if (OccupancyStatus[i][j]>0){
                detectedRoadArea += gridWidth*gridHeight;
            }
        }
    }
    roadOutputFile << frameSeq << ", " << detectedRoadArea << "," <<pointNum << endl;
}



