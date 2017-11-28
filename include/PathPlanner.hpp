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
    double timeToGoal;
    bool initialized;
    double curShortestTime;
    double curATime;
    gridInfo* shortestNext;
    gridInfo* shortestPrev;
    bool considered;
    gridInfo* me;

    bool operator < ( const gridInfo gridInfoB) const{
        return this->curATime > gridInfoB.curATime;
    }
};


class PathPlanner {

    std::vector<gridInfo*> grids;

    OccupancyGrid* mOccupancyGrid;

    std::vector<std::vector<bool>> route;


public:
    PathPlanner(double gridWidth,double gridHeight,double XMin,double XMax,double ZMin,double ZMax);
    ~PathPlanner();
    gridInfo* createAddCleanGridInfo(int x, int z);
    gridInfo* searchGridInfo(int x, int z);

    void ClearRoute();

    void
    PlanPath_AStar_ManualRoadModel(cv::Mat &pc, sl::Mat &slpc, double srcX, double srcZ, double dstX, double dstZ, float A,
                                   float B, float C, float D, double HorizonZMax,double HorizonZMin);

    bool AStar(double srcX, double srcZ, double dstX, double dstZ, double HorizonZMax,double HorizonZMin);

    bool IsValidGridWayPointWithEnoughSpaceForCar(gridInfo *gridNode);

    void ResetGrids();

    bool IsValidGridWayPointWithEnoughSpaceForCar(int x, int y);
};


#endif //PROJECT_PATHPLANNER_H
