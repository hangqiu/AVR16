//
// Created by nsl on 9/17/16.
//

#include "DeepCam/include/Displayer.h"


int Displayer::displayExistingObjOnPrevIm(int R, int G, int B){
    Point2f vertices[4];
    for (int rect_idx=0;rect_idx < int(camera->getExisting_obj().size());rect_idx++){
        if (camera->getExisting_obj()[rect_idx].alive){

            vertices[0].x = camera->getExisting_obj()[rect_idx].box.left;
            vertices[1].x = camera->getExisting_obj()[rect_idx].box.left;
            vertices[2].x = camera->getExisting_obj()[rect_idx].box.right;
            vertices[3].x = camera->getExisting_obj()[rect_idx].box.right;
            vertices[0].y = camera->getExisting_obj()[rect_idx].box.bot;
            vertices[1].y = camera->getExisting_obj()[rect_idx].box.top;
            vertices[2].y = camera->getExisting_obj()[rect_idx].box.top;
            vertices[3].y = camera->getExisting_obj()[rect_idx].box.bot;
            for (int i = 0; i < 4; i++)
            {
                line(camera->getPrevIm(), vertices[i], vertices[(i+1)%4], Scalar(R,G,B),3);
            }
            char tmp[30];
            sprintf(tmp,"%s%d:%s", camera->getExisting_obj()[rect_idx].box.type, camera->getExisting_obj()[rect_idx].id,camera->getExisting_obj()[rect_idx].status.c_str());
            // sprintf(tmp,"%s", camera->getExisting_obj()[rect_idx].box.type);
            putText(camera->getPrevIm(), tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
            if (DEBUG) printf("displaying box: %d,%d,%d,%d\n", camera->getExisting_obj()[rect_idx].box.left,camera->getExisting_obj()[rect_idx].box.right,camera->getExisting_obj()[rect_idx].box.top,camera->getExisting_obj()[rect_idx].box.bot);
        }
        // putText(im, obj_bb_box[rect_idx].score, vertices[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
    }
    int key;
    if (SHOW_IMG) {
        imshow(WIN_NAME, camera->getPrevIm());
        key = cv::waitKey(5);
    }
//    if (WRITE_VIDEO){ outputVideo<<curIm;}

    return key;
}


void Displayer::drawKPTMatchesOnPrevIm(){
    for( int i = 0; i < multiObjTracker->getPoints()[1].size(); i++ )
    {
        if( multiObjTracker->getStatus()[i] ){
            circle( camera->getPrevIm(), multiObjTracker->getPoints()[0][i], 3, Scalar(255,0,0), -1, 8);
            circle( camera->getPrevIm(), multiObjTracker->getPoints()[1][i], 3, Scalar(0,255,0), -1, 8);
            line( camera->getPrevIm(), multiObjTracker->getPoints()[0][i], multiObjTracker->getPoints()[1][i], Scalar(0,0,255));
        }
    }
}

void Displayer::showOriginalIm(string basic_string) {
    imshow(basic_string, camera->getIm_0());
}


void Displayer::showCurIm(string basic_string) {
    imshow(basic_string, camera->getCurIm());
}

Displayer::Displayer(Cam *camera, MultiObjTracker *multiObjTracker) : camera(camera),
                                                                      multiObjTracker(multiObjTracker) {}

Displayer::~Displayer() {

}

//
//char Displayer::display_multi(Mat im, vector<CMT> & cmt_vec, vector<Object> camera->getExisting_obj())
//{
//    //Visualize the output
//    //It is ok to draw on im itself, as CMT only uses the grayscale image
//    const bool multiObjTracker->getPoints() = true;
//    Point2f vertices[4];
//    for (int rect_idx=0;rect_idx < int(cmt_vec.size());rect_idx++){
//
//        if (multiObjTracker->getPoints()){
//            for(size_t i = 0; i < cmt_vec[rect_idx].multiObjTracker->getPoints()_active.size(); i++)
//            {
//                cv::circle(im, cmt_vec[rect_idx].multiObjTracker->getPoints()_active[i], 2, Scalar(255,0,0));
//            }
//        }
//        cmt_vec[rect_idx].bb_rot.multiObjTracker->getPoints()(vertices);
//        for (int i = 0; i < 4; i++)
//        {
//            if (camera->getExisting_obj()[rect_idx].alive){
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
//            }else{
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(0,0,255));
//            }
//        }
//        // vertices[0].x = camera->getExisting_obj()[rect_idx].box.left;
//        // vertices[1].x = camera->getExisting_obj()[rect_idx].box.left;
//        // vertices[2].x = camera->getExisting_obj()[rect_idx].box.right;
//        // vertices[3].x = camera->getExisting_obj()[rect_idx].box.right;
//        // vertices[0].y = camera->getExisting_obj()[rect_idx].box.bot;
//        // vertices[1].y = camera->getExisting_obj()[rect_idx].box.top;
//        // vertices[2].y = camera->getExisting_obj()[rect_idx].box.top;
//        // vertices[3].y = camera->getExisting_obj()[rect_idx].box.bot;
//        // for (int i = 0; i < 4; i++)
//        // {
//        //     line(im, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
//        // }
//        char tmp[30];
//        sprintf(tmp,"%s%d", camera->getExisting_obj()[rect_idx].box.type, camera->getExisting_obj()[rect_idx].id);
//        putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//        // putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//        // putText(im, obj_bb_box[rect_idx].score, vertices[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//
//    }
//
//    if (SHOW_IMG) {imshow(WIN_NAME, im);}
//    if (WRITE_VIDEO){ outputVideo<<im;}
//
//    return waitKey(5);
//}
//
//int display_2_obj(Mat im, vector<Object> camera->getExisting_obj(), vector<Object> moved_obj)
//{
//    Point2f vertices[4];
//    for (int rect_idx=0;rect_idx < int(camera->getExisting_obj().size());rect_idx++){
//        if (camera->getExisting_obj()[rect_idx].alive){
//
//            vertices[0].x = camera->getExisting_obj()[rect_idx].box.left;
//            vertices[1].x = camera->getExisting_obj()[rect_idx].box.left;
//            vertices[2].x = camera->getExisting_obj()[rect_idx].box.right;
//            vertices[3].x = camera->getExisting_obj()[rect_idx].box.right;
//            vertices[0].y = camera->getExisting_obj()[rect_idx].box.bot;
//            vertices[1].y = camera->getExisting_obj()[rect_idx].box.top;
//            vertices[2].y = camera->getExisting_obj()[rect_idx].box.top;
//            vertices[3].y = camera->getExisting_obj()[rect_idx].box.bot;
//            for (int i = 0; i < 4; i++)
//            {
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
//            }
//            char tmp[30];
//            sprintf(tmp,"%s%d", camera->getExisting_obj()[rect_idx].box.type, camera->getExisting_obj()[rect_idx].id);
//            putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//            vertices[0].x = moved_obj[rect_idx].box.left;
//            vertices[1].x = moved_obj[rect_idx].box.left;
//            vertices[2].x = moved_obj[rect_idx].box.right;
//            vertices[3].x = moved_obj[rect_idx].box.right;
//            vertices[0].y = moved_obj[rect_idx].box.bot;
//            vertices[1].y = moved_obj[rect_idx].box.top;
//            vertices[2].y = moved_obj[rect_idx].box.top;
//            vertices[3].y = moved_obj[rect_idx].box.bot;
//            for (int i = 0; i < 4; i++)
//            {
//                line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
//            }
//            // char tmp[30];
//            // sprintf(tmp,"%s%d", camera->getExisting_obj()[rect_idx].box.type, camera->getExisting_obj()[rect_idx].id);
//            // putText(im, tmp, vertices[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//        }
//        // putText(im, obj_bb_box[rect_idx].score, vertices[2], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,200,200), 4);
//
//    }
//
//    if (SHOW_IMG) {imshow(WIN_NAME, im);}
//    if (WRITE_VIDEO){ outputVideo<<im;}
//
//    return waitKey(1000);
//}
//
