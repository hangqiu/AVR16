//
// Created by nsl on 9/17/16.
//

#include <obj_record.pb.h>
#include "DeepCam/include/IO.h"
#include "DeepCam/MPEG7_Lib/Frame.h"
#include "DeepCam/MPEG7_Lib/FexWrite.h"


//IO::IO(char* DatasetPath, char* CamID, char* metadata_path, Cam* myCam){
IO::IO( char* DatasetPath, char* CamID,char* metadata_path){
    camera = NULL;
    //open metadata
    metadata_file.open(metadata_path);
//    metadata_file.open(metadata_path);
    if(!metadata_file.is_open()){
        cerr<<"unable to open metadata."<< endl;
        exit (-1);
    }
    // if (ORIENTATION_SANITY_CHECK){
//    opflow_file.open("./op_flow.txt");
//    if(!obj_file.is_open()){
//        cerr<<"unable to open output file op_flow.txt"<< endl;
//        exit (-1);
//    }

    char output_path[50];
    if (TRACK_EVAL_OUTPUT){
        sprintf(output_path,"./%s/cam%s_track_eval.txt",DatasetPath,CamID);
    }else{
        sprintf(output_path,"./%s/cam%s_obj_list.txt",DatasetPath,CamID);
    }
    obj_file.open(output_path);
    if(!obj_file.is_open()){
        cerr<<"unable to open output file " << output_path<< endl;
        exit (-1);
    }
    char tmp_path[50];
    sprintf(tmp_path,"./%s/cam%s_yolo_box.txt",DatasetPath,CamID);
    yolo_obj_file.open(tmp_path);
    sprintf(tmp_path,"./%s/cam%s_latency.txt",DatasetPath,CamID);
    latency_file.open(tmp_path);
    sprintf(tmp_path,"./%s/cam%s_total_latency.txt",DatasetPath,CamID);
    total_latency_file.open(tmp_path);

    sprintf(outputPrefix, "./%s/%s", DatasetPath, CamID);

}

void IO::setCamMeta(Cam* myCam){
    camera = myCam;

    long curTS;
    int fps;
    double azimuth;
    double lat, lon;
    metadata_file >> curTS >> fps >> azimuth >> lat >> lon;
    camera->setCurTS(curTS);
    camera->setFps(fps);
    camera->setAzimuth(azimuth);
    camera->setLat(lat);
    camera->setLon(lon);

    // TODO make it camera specifi variables.
    MOVE_CENTROID_THRESH = MOVE_CENTROID_THRESH / (fps/10); //since it's 30 fps now
    STOP_CENTROID_THRESH = STOP_CENTROID_THRESH / (fps/10); //since it's 30 fps now
}

void IO::initProtobuf(char* CamID, double lat, double lon, char* input_path){
    obj_book.set_camid(CamID);
    obj_book.set_lat(lat);
    obj_book.set_lon(lon);
    obj_book.set_videopath(input_path);
}

void IO::writeOriginalFrame(char* filepath){
    imwrite(filepath,camera->getIm_0());
}

void IO::writeResizedFrame(char* filepath){
    imwrite(filepath,camera->getCurIm());
}

void IO::printExistingObj(){
    // cout << "Printing object vector:\n";
    for (int new_idx = 0;(new_idx<int(camera->getExisting_obj().size())&&(camera->getExisting_obj()[new_idx].box.left!=YOLO_MARK));new_idx++){
        if (DEBUG) printf("Object # %d, [left,top,right,bot]:%d %d %d %d\n", camera->getExisting_obj()[new_idx].id,camera->getExisting_obj()[new_idx].box.left,camera->getExisting_obj()[new_idx].box.top,camera->getExisting_obj()[new_idx].box.right,camera->getExisting_obj()[new_idx].box.bot);
    }
}

void IO::logObjects(){
    tracklets::Snapshot* snapshot_pt = obj_book.add_snapshot();
    snapshot_pt->set_frameid(camera->getCurFrameId());
    // snapshot_pt->set_timestamp(meta_data.timestamp);
    // snapshot_pt->set_lat(meta_data.lat);
    // snapshot_pt->set_lon(meta_data.lon);
    snapshot_pt->set_timestamp(camera->getFps());

    for (int idx = 0;idx<int(camera->getExisting_obj().size())&&(camera->getExisting_obj()[idx].box.left!=YOLO_MARK);idx++){
        // alive objects only
        if (camera->getExisting_obj()[idx].alive) {
            // if ( ( camera->getExisting_obj()[idx].status.compare(MOVE)==0 && frame_id%(int(fps*OUTPUT_FREQ)) == 0 ) || camera->getExisting_obj()[idx].status.compare(ENTER) ==0){
            // int frame_thresh = fps*OUTPUT_FREQ;
            // if ( (frame_id % frame_thresh == 0 ) || (camera->getExisting_obj()[idx].status.compare(ENTER) ==0 ) ) {
            // log object entered, or moving every 1 second
            if (DEBUG) cout << "Logging " << camera->getExisting_obj()[idx].status;
            if (DEBUG) printf(" object %d at frame id %d / %d / %d\n" , camera->getExisting_obj()[idx].id, camera->getCurFrameId(), int(camera->getFps()*OUTPUT_FREQ),camera->getCurFrameId()%(int(camera->getFps()*OUTPUT_FREQ)));
            // averaging the moving direction of fps*OUTPUT_FREQ number of frames
            // if (camera->getExisting_obj()[idx].status.compare(MOVE)==0){
            //     camera->getExisting_obj()[idx].displacement = get_centroid_distance(camera->getExisting_obj()[idx].last_box, camera->getExisting_obj()[idx].box);
            //     // printf("getting centroid distance from box, %d %d %d %d,  and box,%d %d %d %d\n centroid distance: %f\n", camera->getExisting_obj()[idx].box.left,camera->getExisting_obj()[idx].box.top,camera->getExisting_obj()[idx].box.right,camera->getExisting_obj()[idx].box.bot,camera->getExisting_obj()[idx].last_box.left,camera->getExisting_obj()[idx].last_box.top,camera->getExisting_obj()[idx].last_box.right,camera->getExisting_obj()[idx].last_box.bot,camera->getExisting_obj()[idx].displacement);
            //     camera->getExisting_obj()[idx].direction = get_centroid_direction(camera->getExisting_obj()[idx].box, camera->getExisting_obj()[idx].last_box);
            //     // printf("getting centroid direction from box, %d %d %d %d,  and box,%d %d %d %d\n centroid direction: %f\n", camera->getExisting_obj()[idx].box.left,camera->getExisting_obj()[idx].box.top,camera->getExisting_obj()[idx].box.right,camera->getExisting_obj()[idx].box.bot,camera->getExisting_obj()[idx].last_box.left,camera->getExisting_obj()[idx].last_box.top,camera->getExisting_obj()[idx].last_box.right,camera->getExisting_obj()[idx].last_box.bot,camera->getExisting_obj()[idx].direction);
            // }
            // update last box
//            camera->getExisting_obj()[idx].last_box = camera->getExisting_obj()[idx].box;
            camera->getExisting_obj()[idx].setLast_box(camera->getExisting_obj()[idx].box);
            /////////////////////////// getting thumbnail/////////////////////////
            Rect rect_tmp = Rect(camera->getExisting_obj()[idx].box.left,camera->getExisting_obj()[idx].box.top,camera->getExisting_obj()[idx].box.right-camera->getExisting_obj()[idx].box.left,camera->getExisting_obj()[idx].box.bot-camera->getExisting_obj()[idx].box.top);
            rect_tmp = legalize_box(camera->getCurIm(),rect_tmp);
            Mat obj_tmp;
//            if (SHOW_KYPTS_THUMBNAIL){ // TODO
//                obj_tmp = im(rect_tmp);
//            }else{
                obj_tmp = camera->getCurIm()(rect_tmp);
//            }
            char tmpstr[200];
            sprintf(tmpstr, "./%s_%d_%lu_%d.jpg", outputPrefix, camera->getCurFrameId(), camera->getCurTS(),camera->getExisting_obj()[idx].id);//camID(videopath)_timestamp_objID.jpg
            imwrite(tmpstr,obj_tmp);
            ////////////////////////////// eval vision feature ////////////////////////
            // int tmp_start = cvGetTickCount();
            // vector<KeyPoint> tmp_kypts = detect_keypoints(get_gray(obj_tmp));
            // int tmp_mid = cvGetTickCount();
            // points[0] = convert_keypoints_to_point(tmp_kypts);
            // double tmp_detect_time = (double)(tmp_mid-tmp_start)/ (cvGetTickFrequency()) / 1000000;
            // double tmp_describe_time= 0;
            // Mat tmp_described_features;
            // int tmp_end;
            // size_t tmp_sizeInBytes;
            // if (FEATURE_TYPE=="ORB" || FEATURE_TYPE=="BRISK" ){
            //     tmp_described_features= describe_keypoints(tmp_kypts,get_gray(obj_tmp));
            //     tmp_end = cvGetTickCount();
            //     tmp_sizeInBytes= tmp_described_features.total() * tmp_described_features.elemSize();
            //     tmp_describe_time = (double)(tmp_end-tmp_mid)/ (cvGetTickFrequency()) / 1000000;
            //     if (TRACK_EVAL_OUTPUT){
            //         latency_file << frame_id << ": "<< tmp_detect_time << "+" << tmp_describe_time << "=" << tmp_detect_time+tmp_describe_time  << ", size:" << tmp_sizeInBytes<< endl;
            //     }
            // }

            ///////////////////////// file and PROTOBUF logging objects//////////////////////
            if (TRACK_EVAL_OUTPUT){
                obj_file << camera->getCurFrameId() << ", "<< camera->getExisting_obj()[idx].id << ", " << camera->getExisting_obj()[idx].box.left << ", "<< camera->getExisting_obj()[idx].box.top << ", "
                         << camera->getExisting_obj()[idx].box.right - camera->getExisting_obj()[idx].box.left << ", " << camera->getExisting_obj()[idx].box.bot - camera->getExisting_obj()[idx].box.top
                         << ", -1, -1, -1, -1" << endl;
            }else{
                obj_file << camera->getCamID()  <<  " " << camera->getCurFrameId() << " " << camera->getCurTS() <<" " << camera->getLat()<< " "<< camera->getLon()<< " "
                         << camera->getExisting_obj()[idx].id << " " << camera->getExisting_obj()[idx].alive << " " << camera->getExisting_obj()[idx].status << " "
                         << camera->getExisting_obj()[idx].direction <<" " << camera->getExisting_obj()[idx].displacement << " " << tmpstr;

            }
            tracklets::Snapshot::Object* proto_obj = snapshot_pt->add_object();
            proto_obj->set_objid(camera->getExisting_obj()[idx].id);
            proto_obj->set_thumbnailpath(tmpstr);
            proto_obj->set_status(camera->getExisting_obj()[idx].status);
            proto_obj->set_view(camera->getExisting_obj()[idx].view);

            // grab cut thumbnail
            if (GRABCUT_FLAG){
                sprintf(tmpstr, "./%s_%d_%lu_%d_grabcut.jpg", outputPrefix, camera->getCurFrameId(), camera->getCurTS(), camera->getExisting_obj()[idx].id);//camID(videopath)_timestamp_objID.jpg
                obj_tmp = grabcut_thumbnail(obj_tmp);
                imwrite(tmpstr,obj_tmp);
                proto_obj->set_grabcutpath(tmpstr);
            }

            proto_obj->set_direction(camera->getExisting_obj()[idx].direction);
            proto_obj->set_displacement(camera->getExisting_obj()[idx].displacement);
            //////////////////////////////////////// color layout descriptor////////////////////////////////////////
//            cout << "Object No. " << camera->getExisting_obj()[idx].id << " " << tmpstr << "\n";

            sprintf(tmpstr, "./%s_%lu_%d.txt", outputPrefix, camera->getCurTS(),camera->getExisting_obj()[idx].id);
            writeCLD(obj_tmp,tmpstr);
            ///////////////////////// file and PROTOBUF logging objects//////////////////////
            if (!TRACK_EVAL_OUTPUT){
                obj_file << " " << tmpstr << endl;
            }
            proto_obj->set_cldpath(tmpstr);
            //type, switch string is not supported
            proto_obj->set_objtype(camera->getExisting_obj()[idx].box.type);
            // if (strcmp(camera->getExisting_obj()[idx].box.type, "Car")){proto_obj->set_objtype(tracklets::Snapshot::Car);}
            // else if (strcmp(camera->getExisting_obj()[idx].box.type, "Person")){proto_obj->set_objtype(tracklets::Snapshot::Person);}
            // else {proto_obj->set_objtype(tracklets::Snapshot::Unknown);}
            // }
        }
    }
}


void IO::writeCLD(Mat img, char* filepath){
    Frame* frame = new Frame(img.cols, img.rows, true, true, true);
    // set the image of the frame
    frame->setImage(img);
    // CLD with numberOfYCoeff (28), numberOfCCoeff (15)
    // int start = cvGetTickCount();
    FexWrite::computeWriteCLD( frame, 28, 15 , filepath );
    // int mid = cvGetTickCount();
    // double detect_time = (double)(mid-start)/ (cvGetTickFrequency()) / 1000000;
    // cout << endl << detect_time << endl;
    // release frame
    delete frame;
    return;
}

void IO::writeProtobuf(char* proto_path){
    fstream final_output(proto_path, ios::out | ios::trunc | ios::binary);
    if (!obj_book.SerializeToOstream(&final_output)) {
        cerr << "Failed to write ObjBook." << endl;
        google::protobuf::ShutdownProtobufLibrary();
        exit (-1);
    }
    cout << "Write protobuf output to " <<  proto_path << "\n"<< endl;
}

IO::~IO() {

}
