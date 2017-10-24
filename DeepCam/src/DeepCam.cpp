//
// Created by nsl on 9/16/16.
//

#include "DeepCam/include/DeepCam.h"




//void Cam::init(string input_path,string DatasetPath, char* CamID,char* metadata_path,char* cfgfile,char* weightfile, IO* io) {
DeepCam::DeepCam(string input_path, string& DatasetPath, string& CamID,string& metadata_path, string& cfgfile, string& weightfile, IO* io){
    curFrameId = 0;
//    curTS = 0;

    id = CamID;
// open video
    //If no input was specified
    if (input_path.length() == 0){cap.open(0);}
        //Else open the video specified by input_path
    else{cap.open(input_path);}

    //If it doesn't work, stop
    if(!cap.isOpened()){
        cerr << "Unable to open video capture." << endl;
        exit(-1);
    }

    // setting flags
    if (DEBUG) PAUSE_FLAG = true; // start from pausing....in debug mode

    multiObjTracker = new MultiObjTracker(this);
    multiObjMatcher = new MultiObjMatcher(multiObjTracker, this, io);

    io->setCamMeta(this);
// yolo has to be initialized last, since it changes this pointer
    yolo = new YOLO();
    yolo->init(cfgfile,weightfile);

}


bool DeepCam::loadFrame() {


    cap >> im_0;
    if (im_0.empty()) return false;
    if (curFrameId==0){ // initial call
        scale = im_0.rows / RESIZEROWS;
        rows = im_0.rows;
        cols = im_0.cols;
        resizeRows = curIm.rows;
        resizeCols = curIm.cols;
    }

    curIm.copyTo(prevIm);
    curGray.copyTo(prevGray);

    curIm = resize_Input_color(im_0,scale);
    curGray = get_gray(curIm);

    if (DEBUG) printf("Loaded new frame %d\n", curFrameId);

    curFrameId++;
    curTS = curTS+int(1000/fps);

    //debug
    return true;
}

Mat DeepCam::resize_Input_color(Mat im_in, double scale){
    Mat im0( cvRound (im_in.rows/scale),cvRound(im_in.cols/scale), CV_8UC3);
    resize(im_in,im0,im0.size(),0, 0, INTER_LINEAR);
    return im0;
}

Mat DeepCam::resize_Input_gray(Mat im_in, double scale){
    Mat im0( cvRound (im_in.rows/scale),cvRound(im_in.cols/scale), CV_8UC1);
    resize(im_in,im0,im0.size(),0, 0, INTER_LINEAR);
    return im0;
}

Mat DeepCam::get_gray(Mat im0){
    Mat im0_gray;
    if (im0.channels() > 1) {
        cvtColor(im0, im0_gray, CV_BGR2GRAY);
    } else {
        im0_gray = im0;
    }
    return im0_gray;
}



bool DeepCam::process_key(char key){
    // pausing
    if (key == 'p' || PAUSE_FLAG) pause_step_resume();
//    if (key == 's') save_img();
    // quiting
    if(key == 'q') {
        return true;
    }
    return false;
}

void DeepCam::pause_step_resume(){
    if (DEBUG) cout << "pausing....\n";
    char new_key;

    while(new_key != 'r'){
        new_key = cv::waitKey(20);
        if (new_key == 'n') {
            PAUSE_FLAG = true;
            if (DEBUG) cout << "resuming....\n";
            return;
        }
//        if (new_key == 's') {
//            save_img();
//        }
    }
    if (DEBUG) cout << "resuming....\n";
    PAUSE_FLAG = false;
}


//TODO: simplify and modularize
void DeepCam::MultiObjMatching(){
    // int id = 0;
    // while (true){
    //     cout << id++ << endl;

        // phase 1: compare new box with existing box, filter out not moving
    multiObjMatcher->newObjMotionFilter();
        // phase 2: compare new box with cmt prediction, match moved box, create new box and cmt instance if no match
    multiObjMatcher->matchingMovedObj();
        // phase 3: go through cmt prediction, remove box no longer valid, no match
    multiObjMatcher->updateStaleness();

    evictObj();
}

bool DeepCam::processTrigger() {
    return (curFrameId % YOLO_FREQ == 0);
}

void DeepCam::deadReckoning() {
    existing_obj = moved_obj;
}

int DeepCam::getCurFrameId() const {
    return curFrameId;
}

void DeepCam::evictObj(){
    // mark for eviction
    for (int exist_idx = 0;exist_idx<int(existing_obj.size())&&(existing_obj[exist_idx].box.left!=YOLO_MARK);exist_idx++){
        if (existing_obj[exist_idx].alive == false && existing_obj[exist_idx].evict){
            if (existing_obj[exist_idx].evict_count++ / fps >= EVICT_TRHESH){
                // object eviction for not seen from yolo for a long time.
                if (DEBUG) printf("Erasing Object %d\n", existing_obj[exist_idx].id);
                existing_obj.erase(existing_obj.begin()+exist_idx);
//                if (USE_CMT){
//                    cmt_vec.erase(cmt_vec.begin()+exist_idx);
//                }
                break; // evict one at a time, otherwise idx will not match
            }
        }else{
            existing_obj[exist_idx].evict_count = 0;
        }
    }
}

const Mat &DeepCam::getCurIm() const {
    return curIm;
}

int DeepCam::getRows() const {
    return rows;
}

int DeepCam::getCols() const {
    return cols;
}

int DeepCam::getResizeRows() const {
    return resizeRows;
}

int DeepCam::getResizeCols() const {
    return resizeCols;
}

int DeepCam::getFps() const {
    return fps;
}

double DeepCam::getAzimuth() const {
    return azimuth;
}

double DeepCam::getLat() const {
    return lat;
}

double DeepCam::getLon() const {
    return lon;
}

long DeepCam::getCurTS() const {
    return curTS;
}

char *DeepCam::getCamID() const {
    return (char*)id.c_str();
}

double DeepCam::getScale() const {
    return scale;
}

const DeepCam &Cam::getIm_0() const {
    return im_0;
}

const DeepCam &Cam::getIm_0_gray() const {
    return im_0_gray;
}

const DeepCam &Cam::getCurGray() const {
    return curGray;
}

const DeepCam &Cam::getPrevIm() const {
    return prevIm;
}

const DeepCam &Cam::getPrevGray() const {
    return prevGray;
}


void DeepCam::pushBackExisting_obj(Object object) {
    existing_obj.push_back(object);
}

void DeepCam::setMoved_obj(const vector<Object> &moved_obj) {
    DeepCam::moved_obj = moved_obj;
}

vector<Object> &DeepCam::getExisting_obj() {
    return existing_obj;
}

vector<Object> &DeepCam::getMoved_obj() {
    return moved_obj;
}

vector<Object> &DeepCam::getNew_obj() {
    return new_obj;
}

void DeepCam::setId(char *id) {
    DeepCam::id = id;
}

void DeepCam::setFps(int fps) {
    DeepCam::fps = fps;
}

void DeepCam::setAzimuth(double azimuth) {
    DeepCam::azimuth = azimuth;
}

void DeepCam::setLat(double lat) {
    DeepCam::lat = lat;
}

void DeepCam::setLon(double lon) {
    DeepCam::lon = lon;
}

void DeepCam::setCurTS(long curTS) {
    DeepCam::curTS = curTS;
}


DeepCam::~DeepCam() {

}

DeepCam::DeepCam() {
    yolo = NULL;
    multiObjTracker = NULL;
    multiObjMatcher = NULL;
}
