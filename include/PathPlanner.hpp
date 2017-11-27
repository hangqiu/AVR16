//
// Created by hang on 11/26/17.
//

#ifndef PROJECT_PATHPLANNER_H
#define PROJECT_PATHPLANNER_H

#include "globals.hpp"
#include "OccupancyGrid.hpp"
#include <vector>

struct coords{
    int x;
    int z;
};
class gridInfo{
public:
    coords myCoords;
    int timeToGoal; // Sunday traffic
    bool initialized;
    int curShortestTime;
    int curATime;
    gridInfo* shortestNext;
    gridInfo* shortestPrev;
    bool considered;
    gridInfo* me;

    bool operator < ( const gridInfo gridInfoB) const{
        return this->myCoords.x == gridInfoB.myCoords.x?
               this->myCoords.z < gridInfoB.myCoords.z:
               this->myCoords.x < gridInfoB.myCoords.x;
    }
};


class PathPlanner {

    std::vector<gridInfo*> grids;

    OccupancyGrid* mOccupancyGrid;

    std::vector<std::vector<bool>> route;


public:
    PathPlanner(double gridWidth,double gridHeight,double XMin,double XMax,double ZMin,double ZMax);
    ~PathPlanner();
    gridInfo* createAddGridInfo(int x, int z);
    gridInfo* searchGridInfo(int x, int z);
    bool AStar(int srcX, int srcZ, int dstX, int dstZ);

    void PlanPath_AStar();

    void PlanPath_AStar(cv::Mat &pc, sl::Mat &slpc, int srcX, int srcZ, int dstX, int dstZ);

    void
    PlanPath_AStar_ManualRoadModel(cv::Mat &pc, sl::Mat &slpc, int srcX, int srcZ, int dstX, int dstZ, float A, float B,
                                   float C, float D);

    void ClearRoute();

    void SetRoute();
};


#endif //PROJECT_PATHPLANNER_H
