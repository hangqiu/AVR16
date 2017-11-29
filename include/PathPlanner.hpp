//
// Created by hang on 11/26/17.
//

#ifndef PROJECT_PATHPLANNER_H
#define PROJECT_PATHPLANNER_H

#include "globals.hpp"
#include "OccupancyGrid.hpp"
#include <vector>
#include <queue>

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
private:
    ofstream PathFile;
    ofstream AngleFile;

    std::vector<gridInfo*> grids;

    OccupancyGrid* mOccupancyGrid;

    std::vector<std::vector<bool>> route;


public:
    PathPlanner(double gridWidth,double gridHeight,double XMin,double XMax,double ZMin,double ZMax);
    ~PathPlanner();
    gridInfo* createAddCleanGridInfo(int x, int z);
    gridInfo* searchGridInfo(int x, int z);

    void ClearRoute();

    bool AStar(double srcX, double srcZ, double dstX, double dstZ, double HorizonZMax,double HorizonZMin);

    bool IsValidGridWayPointWithEnoughSpaceForCar(gridInfo *gridNode);

    void ResetGrids();

    bool IsValidGridWayPointWithEnoughSpaceForCar(int x, int y);

    double calcDistance(double xIdx1, double zIdx1, double xIdx2, double zIdx2);

    void iterateNextNode(gridInfo &tmpNode, gridInfo *tmpNextNode, std::priority_queue<gridInfo> &mQueue);

    void
    PlanPath_AStar_ManualRoadModel(int frameSeq, cv::Mat &pc, Mat &slpc, double srcX, double srcZ, double dstX, double dstZ,
                                   float A, float B, float C, float D, double HorizonZMax, double HorizonZMin);

    void OutputAStarPath(int frameSeq, int srcXIdx, int srcZIdx, int dstXIdx, int dstZIdx, int HorizonZMinIdx,
                         int HorizonZMaxIdx);

    void OutputDefaultPath(int frameSeq, int srcXIdx, int srcZIdx, int HorizonZMinIdx, int HorizonZMaxIdx);

    double getRouteAngle();

    double getRouteAngle(int srcXIdx, int srcZIdx);
};


#endif //PROJECT_PATHPLANNER_H
