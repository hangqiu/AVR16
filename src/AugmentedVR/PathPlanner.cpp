//
// Created by hang on 11/26/17.
//

#include <queue>
#include "PathPlanner.hpp"

PathPlanner::PathPlanner(double gridWidth, double gridHeight, double XMin, double XMax, double ZMin, double ZMax) {
    mOccupancyGrid = new OccupancyGrid(gridWidth,gridHeight, XMin, XMax, ZMin, ZMax);
    route.resize(mOccupancyGrid->XNum);
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        route[i].resize(mOccupancyGrid->ZNum);
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            route[i][j]=false;
        }
    }
}

PathPlanner::~PathPlanner() {
    delete mOccupancyGrid;
}

gridInfo* PathPlanner::createAddGridInfo(int x, int z){
    gridInfo* tmpNode = new gridInfo;
    tmpNode->myCoords.x = x;
    tmpNode->myCoords.z = z;
    tmpNode->curShortestTime = 999999999;
    tmpNode->curATime= 999999999;
    tmpNode->shortestNext = NULL;
    tmpNode->shortestPrev = NULL;
    tmpNode->considered = false;
    tmpNode->me=tmpNode;
    tmpNode->initialized = false;
    grids.push_back(tmpNode);
    return tmpNode;
}

gridInfo* PathPlanner::searchGridInfo(int x, int z){
    for (std::vector<gridInfo*>::iterator it = grids.begin() ; it != grids.end(); ++it){
        if ((*it)->myCoords.x==x && (*it)->myCoords.z==z){
            // found the state
            return *it;
        }
    }
    return NULL;
}

void iterateNextNode(gridInfo& tmpNode, gridInfo* tmpNextNode, std::priority_queue<gridInfo>& mQueue){
    int tmpATime = tmpNode.curATime + 1 + tmpNextNode->timeToGoal;
    int tmpActualShortestTime = tmpNode.curShortestTime + 1;
//            int tmpShortestTime = tmpNode->curShortestTime + 1; // BFS assume unit cost

    if (tmpATime < tmpNextNode->curATime){
        tmpNextNode->curShortestTime = tmpActualShortestTime;
        tmpNextNode->curATime = tmpATime;
        tmpNextNode->shortestPrev = (tmpNode.me);
        /// remove the duplicates
        std::vector<gridInfo> tmp;
        while (!mQueue.empty()){
            gridInfo tmpGridNode = mQueue.top();
            mQueue.pop();
            if (tmpGridNode.myCoords.x == tmpNextNode->myCoords.x && tmpGridNode.myCoords.z == tmpNextNode->myCoords.z){
            }else{
                tmp.push_back(tmpGridNode);
            }
        }
        for (int i=0;i<tmp.size();i++){
            mQueue.push(tmp[i]);
        }

        mQueue.push(*tmpNextNode);
    }
}

bool PathPlanner::AStar(int srcX,int srcZ, int dstX, int dstZ){
    /// fill in the info of each grid
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            /// add a node only if the occupancy grid is road, not occupied.
            if (mOccupancyGrid->OccupancyStatus[i][j] > 0){
                gridInfo* tmpNode = createAddGridInfo(i,j);
                tmpNode->timeToGoal = abs(tmpNode->myCoords.x-srcX)+abs(tmpNode->myCoords.z-srcZ);
                tmpNode->initialized = true;
                cout << "valid road grid, " << i << ", " << j << endl;
            }
        }
    }


    std::priority_queue<gridInfo> mQueue;
    gridInfo* start = searchGridInfo(srcX,srcZ);
    if (start==NULL){
        cerr << "invalid src grid" << endl;
        return false;
    }
    start->curShortestTime = 0;
    start->curATime = start->timeToGoal;


    mQueue.push(*start);
    gridInfo tmpNode;
    gridInfo* tmpNextNode;
    while(!mQueue.empty()){
        tmpNode = mQueue.top();
//        tmpNode.considered = true;
        mQueue.pop();
        if (tmpNode.myCoords.x == dstX && tmpNode.myCoords.z == dstZ) break; ///found destination

        tmpNextNode = searchGridInfo(tmpNode.myCoords.x-1,tmpNode.myCoords.z);
        if (tmpNextNode!=NULL) iterateNextNode(tmpNode,tmpNextNode,mQueue);
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x,tmpNode.myCoords.z-1);
        if (tmpNextNode!=NULL) iterateNextNode(tmpNode,tmpNextNode,mQueue);
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x+1,tmpNode.myCoords.z);
        if (tmpNextNode!=NULL) iterateNextNode(tmpNode,tmpNextNode,mQueue);
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x,tmpNode.myCoords.z+1);
        if (tmpNextNode!=NULL) iterateNextNode(tmpNode,tmpNextNode,mQueue);

//        if (DEBUG) cout << tmpNode.name << " " << tmpNode.curShortestTime << endl;
    }
//    if (DEBUG) cout << tmpNode.name << " " << tmpNode.curShortestTime << endl;
    /// set the route
    ClearRoute();

    gridInfo* curNode = searchGridInfo(dstX,dstZ);
    while (curNode->myCoords.x != srcX || curNode->myCoords.z != srcZ){
        int shortestTime = curNode->curShortestTime;
        route[curNode->myCoords.x][curNode->myCoords.z] = true;
        /// not a full path to src
        if (curNode->shortestPrev==NULL) return false;
        curNode = curNode->shortestPrev;
    }
    return true;

}


void PathPlanner::ClearRoute(){
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            route[i][j]=false;
        }
    }
}

void PathPlanner::PlanPath_AStar_ManualRoadModel(cv::Mat &pc, sl::Mat& slpc, int srcX,int srcZ, int dstX, int dstZ,
                                                 float A, float B, float C, float D){

    mOccupancyGrid->ConvertPCAndSetOccupancyGrid_ManualPlaneModel(pc,slpc,A,B,C,D);
    if (!AStar(srcX,srcZ,dstX,dstZ)) return;

    /// color route
    float * p_cloud = slpc.getPtr<float>(sl::MEM_CPU); // Get the pointer
    for (int i=0;i<pc.cols;i++){
        for (int j=0;j<pc.rows;j++){
            int index = (j*pc.cols+i)*4;
            float x = p_cloud[index];
            if (!isValidMeasure(x)) continue;
            float y = p_cloud[index+1];
            float z = p_cloud[index+2];

            int XIdx = int(x-mOccupancyGrid->XMin);
            int ZIdx = int(z-mOccupancyGrid->ZMin);
            if (XIdx<0 || XIdx >= mOccupancyGrid->XNum || ZIdx < 0 || ZIdx >= mOccupancyGrid->ZNum) continue;

            if (A*x+B*y+C*z+D<0 && route[XIdx][ZIdx]){
                /// route on road
                p_cloud[index+3] = 0xFFFF00FF;
            }
        }
    }

}
