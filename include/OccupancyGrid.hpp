//
// Created by hang on 11/26/17.
//

#ifndef PROJECT_OCCUPANCYGRID_H
#define PROJECT_OCCUPANCYGRID_H


#include <vector>
#include "globals.hpp"
#include "AugmentedVR.hpp"
#include <sl/Core.hpp>
#include <opencv2/core/mat.hpp>

class OccupancyGrid {
private:
    ofstream roadOutputFile;
public:
    std::vector<std::vector<int>> OccupancyStatus;/// 0 undefined, positive: road, negative:occupied
    double gridWidth,gridHeight,XMin,XMax,ZMin,ZMax;
    int XNum, ZNum;

public:
    OccupancyGrid(double gridWidth,double gridHeight,double XMin,double XMax,double ZMin,double ZMax);
    ~OccupancyGrid();
    void ConvertPCAndSetOccupancyGrid_ManualPlaneModel(cv::Mat &pc, sl::Mat &slpc, float A, float B, float C, float D);

    void ClearOccupancyStatus();

    void
    ConvertPCAndSetOccupancyGrid_ManualPlaneModel(int frameSeq, cv::Mat &pc, Mat &slpc, float A, float B, float C, float D);
};


#endif //PROJECT_OCCUPANCYGRID_H
