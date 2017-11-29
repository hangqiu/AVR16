//
// Created by hang on 11/26/17.
//


#include "PathPlanner.hpp"
#include <complex>

PathPlanner::PathPlanner(double gridWidth, double gridHeight, double XMin, double XMax, double ZMin, double ZMax) {
    mOccupancyGrid = new OccupancyGrid(gridWidth,gridHeight, XMin, XMax, ZMin, ZMax);
    route.resize(mOccupancyGrid->XNum);
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        route[i].resize(mOccupancyGrid->ZNum);
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            route[i][j]=false;
        }
    }
    PathFile = ofstream("Path.txt");
    AngleFile = ofstream("Angle.txt");
}

PathPlanner::~PathPlanner() {
    delete mOccupancyGrid;
    PathFile.close();
}

gridInfo* PathPlanner::createAddCleanGridInfo(int x, int z){
    gridInfo* tmpNode = searchGridInfo(x,z);
    bool newGrid = (tmpNode==NULL);
    if (newGrid){
        tmpNode = new gridInfo;
    }
    tmpNode->myCoords.x = x;
    tmpNode->myCoords.z = z;
    tmpNode->curShortestTime = 999999999;
    tmpNode->curATime= 999999999;
    tmpNode->shortestNext = NULL;
    tmpNode->shortestPrev = NULL;
    tmpNode->considered = false;
    tmpNode->me=tmpNode;
    tmpNode->initialized = false;
    if (newGrid){
        grids.push_back(tmpNode);
    }
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

double PathPlanner::calcDistance(double xIdx1,double zIdx1,double xIdx2,double zIdx2){
    return sqrt(pow((xIdx1-xIdx2)*mOccupancyGrid->gridWidth,2)+pow((zIdx1-zIdx2)*mOccupancyGrid->gridHeight,2));
}

void PathPlanner::iterateNextNode(gridInfo& tmpNode, gridInfo* tmpNextNode, std::priority_queue<gridInfo>& mQueue){
    double distance = calcDistance(tmpNode.myCoords.x,tmpNode.myCoords.z,tmpNextNode->myCoords.x,tmpNextNode->myCoords.z);
    double tmpATime = tmpNode.curShortestTime + distance + tmpNextNode->timeToGoal;
    double tmpActualShortestTime = tmpNode.curShortestTime + distance;
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


bool PathPlanner::IsValidGridWayPointWithEnoughSpaceForCar(gridInfo *gridNode){

    double carWidth = 2.5;
    double carLength = 4.5;
    int XGrids = int(carWidth/mOccupancyGrid->gridWidth);
    int ZGrids = int(carLength/mOccupancyGrid->gridHeight);
    for (int i=0;i<XGrids;i++){
        for (int j=0;j<ZGrids;j++){
            if (searchGridInfo(gridNode->myCoords.x+i-XGrids/2,gridNode->myCoords.z+j-ZGrids/2)==NULL){
                cout <<gridNode->myCoords.x <<"," << gridNode->myCoords.z << "Not a valid waypoint: "
                     << "not enough space at " << gridNode->myCoords.x+i-XGrids/2 << "," <<gridNode->myCoords.z+j-ZGrids/2 << endl;
                return false;
            }
        }
    }
    return true;
}

bool PathPlanner::IsValidGridWayPointWithEnoughSpaceForCar(int x, int z){

    double carWidth = 2.5;
    double carLength = 4.5;
    int XGrids = int(carWidth/mOccupancyGrid->gridWidth);
    int ZGrids = int(carLength/mOccupancyGrid->gridHeight);
    for (int i=0;i<XGrids;i++){
        for (int j=0;j<ZGrids;j++){
            if (searchGridInfo(x+i-XGrids/2,z+j-ZGrids/2)==NULL){
                cout <<x <<"," << z << "Not a valid waypoint: "
                     << "not enough space at " << x+i-XGrids/2 << "," <<z+j-ZGrids/2 << endl;
                return false;
            }
        }
    }
    return true;
}

void PathPlanner::ResetGrids(){
    for (int i=0;i<grids.size();i++){
        delete grids[i];
    }
    grids.clear();
}

bool PathPlanner::AStar(double srcX,double srcZ, double dstX, double dstZ, double HorizonZMax=18, double HorizonZMin=6){
    int srcXIdx = int((srcX - mOccupancyGrid->XMin) / mOccupancyGrid->gridWidth);
    int srcZIdx = int((srcZ - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    int dstXIdx = int((dstX - mOccupancyGrid->XMin) / mOccupancyGrid->gridWidth);
    int dstZIdx = int((dstZ - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    int HorizonZMaxIdx = int((HorizonZMax - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    int HorizonZMinIdx = int((HorizonZMin - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    //re-init grids
    ResetGrids();
    /// fill in the info of each grid
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        /// adaptive ZMinIdx, assume road until the first grid of detected road
        bool firstRoadGridOfThisColumn = false;
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            /// add a node only if the occupancy grid is road, not occupied.
            /// or beyond sensing range, which is assumed road
//            if (mOccupancyGrid->OccupancyStatus[i][j] > 0 || j>HorizonZMaxIdx || j< HorizonZMinIdx){
            if (mOccupancyGrid->OccupancyStatus[i][j] > 0 || j>HorizonZMaxIdx || !firstRoadGridOfThisColumn){
                gridInfo* tmpNode = createAddCleanGridInfo(i, j);
                tmpNode->timeToGoal = calcDistance(tmpNode->myCoords.x,tmpNode->myCoords.z, dstXIdx,dstZIdx);
//                tmpNode->timeToGoal = pow((tmpNode->myCoords.x-dstXIdx)*mOccupancyGrid->gridWidth,2)+
//                                      pow((tmpNode->myCoords.z-dstZIdx)*mOccupancyGrid->gridHeight,2);
                tmpNode->initialized = true;
                cout << "valid road grid, " << i << ", " << j << endl;
                if (mOccupancyGrid->OccupancyStatus[i][j] > 0){
                    /// mark first road detected, no tolerance anymore
                    firstRoadGridOfThisColumn = true;
                }
            }
        }
    }


    std::priority_queue<gridInfo> mQueue;
    gridInfo* start = searchGridInfo(srcXIdx,srcZIdx);
    if (start==NULL){
        cerr << "invalid src grid" << endl;
        return false;
    }
    start->curShortestTime = 0;
    start->curATime = start->timeToGoal;


    mQueue.push(*start);
    gridInfo tmpNode;
    gridInfo* curNode_ptr;
    gridInfo* tmpNextNode;
    while(!mQueue.empty()){
        tmpNode = mQueue.top();
        curNode_ptr = searchGridInfo(tmpNode.myCoords.x,tmpNode.myCoords.z);
        mQueue.pop();
        if (tmpNode.myCoords.x == dstXIdx && tmpNode.myCoords.z == dstZIdx) break; ///found destination

        tmpNextNode = searchGridInfo(tmpNode.myCoords.x,tmpNode.myCoords.z+1);
        if (tmpNextNode!=NULL && IsValidGridWayPointWithEnoughSpaceForCar(tmpNextNode)) {
            iterateNextNode(*curNode_ptr,tmpNextNode,mQueue);
        }
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x+1,tmpNode.myCoords.z+1);
        if (tmpNextNode!=NULL && IsValidGridWayPointWithEnoughSpaceForCar(tmpNextNode)) {
            iterateNextNode(*curNode_ptr,tmpNextNode,mQueue);
        }
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x-1,tmpNode.myCoords.z+1);
        if (tmpNextNode!=NULL && IsValidGridWayPointWithEnoughSpaceForCar(tmpNextNode)) {
            iterateNextNode(*curNode_ptr,tmpNextNode,mQueue);
        }
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x-1,tmpNode.myCoords.z);
        if (tmpNextNode!=NULL && IsValidGridWayPointWithEnoughSpaceForCar(tmpNextNode)) {
            iterateNextNode(*curNode_ptr,tmpNextNode,mQueue);
        }
        tmpNextNode = searchGridInfo(tmpNode.myCoords.x+1,tmpNode.myCoords.z);
        if (tmpNextNode!=NULL && IsValidGridWayPointWithEnoughSpaceForCar(tmpNextNode)) {
            iterateNextNode(*curNode_ptr,tmpNextNode,mQueue);
        }

        cout << "CurrentNode: (" << tmpNode.myCoords.x << ", " << tmpNode.myCoords.z << "), Shortest Time: " << tmpNode.curShortestTime << endl;
    }
//    if (DEBUG) cout << tmpNode.name << " " << tmpNode.curShortestTime << endl;

    return true;
}


void PathPlanner::ClearRoute(){
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            route[i][j]=false;
        }
    }
}

void PathPlanner::OutputAStarPath(int frameSeq, int srcXIdx, int srcZIdx, int dstXIdx,int dstZIdx, int HorizonZMinIdx, int HorizonZMaxIdx){
    /// set the route
    ClearRoute();

    /// mark route from dst-- Note: it can only trace backward
    /// output path to file
    gridInfo* curNode = searchGridInfo(dstXIdx,dstZIdx);
    gridInfo* lastNode = NULL;
    if (curNode==NULL) {
        cerr << "Dest Idx Invalid" << endl;
        OutputDefaultPath(frameSeq, srcXIdx, srcZIdx,HorizonZMinIdx, HorizonZMaxIdx);
        return;
    }
    /// output path, record route
    PathFile << "Frame, " << frameSeq;
    while (curNode->myCoords.x != srcXIdx || curNode->myCoords.z != srcZIdx) {
        if (lastNode != NULL && curNode->shortestPrev == lastNode) {
            cerr << "Loop Path Terminated" << endl;
            break;
        }
        int shortestTime = curNode->curShortestTime;
        route[curNode->myCoords.x][curNode->myCoords.z] = true;
        PathFile << ", "<< curNode->myCoords.x << "," << curNode->myCoords.z;
        /// not a full path to src
        if (curNode->shortestPrev==NULL) {
            cerr << "not a full path to src" << endl;
            PathFile << ",";
            OutputDefaultPath(frameSeq,srcXIdx, srcZIdx, HorizonZMinIdx, HorizonZMaxIdx);return;
        }
        lastNode = curNode;
        curNode = curNode->shortestPrev;
    }
    PathFile << endl;
}


double PathPlanner::getRouteAngle(int srcXIdx, int srcZIdx){
    int farthestValidZIdx=0;
    int farthestValidXIdx=0;
    for (int i=0;i<mOccupancyGrid->XNum;i++){
        for (int j=0;j<mOccupancyGrid->ZNum;j++){
            if (route[i][j] && mOccupancyGrid->OccupancyStatus[i][j]>0){
                if (j > farthestValidZIdx){
                    farthestValidZIdx = j;
                    farthestValidXIdx = i;
                }
            }
        }
    }
    double angle = atan(double(farthestValidXIdx - srcXIdx) / double(farthestValidZIdx - srcZIdx)) * 180 / PI;
    return angle;
}

/// default path search for the nearest blocking grid zidx
void PathPlanner::OutputDefaultPath(int frameSeq, int srcXIdx , int srcZIdx, int HorizonZMinIdx, int HorizonZMaxIdx){
    ClearRoute();
    int routeZIdx = mOccupancyGrid->ZNum;
    /// search for the nearest blocking grid zidx
    for (int z=HorizonZMinIdx+1;z<mOccupancyGrid->ZNum;z++){
        if ((mOccupancyGrid->OccupancyStatus[srcXIdx][z] < 0 || !IsValidGridWayPointWithEnoughSpaceForCar(srcXIdx,z)) && z < HorizonZMaxIdx){
            routeZIdx = z;
//            routeZIdx = z-int(3/mOccupancyGrid->gridHeight);
            break;
        }
    }
    for (int z=srcZIdx;z<routeZIdx;z++){
        route[srcXIdx][z] =true;
    }
    PathFile << "Frame, "<<frameSeq << "," << "Default"<< endl;
}

void PathPlanner::PlanPath_AStar_ManualRoadModel(int frameSeq, cv::Mat &pc, sl::Mat& slpc, double srcX,double srcZ, double dstX, double dstZ,
                                                 float A, float B, float C, float D, double HorizonZMax, double HorizonZMin){
    int srcXIdx = int((srcX - mOccupancyGrid->XMin) / mOccupancyGrid->gridWidth);
    int srcZIdx = int((srcZ - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    int dstXIdx = int((dstX - mOccupancyGrid->XMin) / mOccupancyGrid->gridWidth);
    int dstZIdx = int((dstZ - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    int HorizonZMaxIdx = int((HorizonZMax - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
    int HorizonZMinIdx = int((HorizonZMin - mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);

    mOccupancyGrid->ConvertPCAndSetOccupancyGrid_ManualPlaneModel(frameSeq, pc,slpc,A,B,C,D);
    /// a star can only trace backward, possibly swap src and dst
    if (!AStar(srcX,srcZ,dstX,dstZ, HorizonZMax, HorizonZMin)) {
        cerr << "Can't find a path" << endl;
        OutputDefaultPath(frameSeq, srcXIdx, srcZIdx,  HorizonZMinIdx,  HorizonZMaxIdx);
    }else{
        OutputAStarPath( frameSeq, srcXIdx,  srcZIdx,  dstXIdx, dstZIdx, HorizonZMinIdx, HorizonZMaxIdx);
    }

    double angle = getRouteAngle(srcXIdx, srcZIdx);
    AngleFile << "Frame, " << frameSeq << "," << angle << endl;
    /// color route
    float * p_cloud = slpc.getPtr<float>(sl::MEM_CPU); // Get the pointer
    for (int i=0;i<pc.cols;i++){
        for (int j=0;j<pc.rows;j++){
            int index = (j*pc.cols+i)*4;
            float x = p_cloud[index];
            if (!isValidMeasure(x)) continue;
            float y = p_cloud[index+1];
            float z = p_cloud[index+2];

            int XIdx = int((x-mOccupancyGrid->XMin) / mOccupancyGrid->gridWidth);
            int ZIdx = int((z-mOccupancyGrid->ZMin) / mOccupancyGrid->gridHeight);
            if (XIdx<0 || XIdx >= mOccupancyGrid->XNum || ZIdx < 0 || ZIdx >= mOccupancyGrid->ZNum) continue;


            if (A*x+B*y+C*z+D<0 && route[XIdx][ZIdx]){
                /// route on road
                /// color a narrower path
                double narrowRatio = 10;
                if ( (x > mOccupancyGrid->XMin + mOccupancyGrid->gridWidth * (XIdx + 0.5-1/narrowRatio) && x < mOccupancyGrid->XMin + mOccupancyGrid->gridWidth * (XIdx + 0.5+1/narrowRatio)) ||
                     (z > mOccupancyGrid->ZMin + mOccupancyGrid->gridHeight * (ZIdx + 0.5-1/narrowRatio) && z < mOccupancyGrid->ZMin + mOccupancyGrid->gridHeight * (ZIdx + 0.5+1/narrowRatio)))
                {
                    p_cloud[index+3] = 0xFFFF00FF;
                }
            }
        }
    }
}
