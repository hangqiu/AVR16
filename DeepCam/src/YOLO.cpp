//
// Created by nsl on 9/16/16.
//

#include "DeepCam/include/YOLO.h"

#include <iostream>



    void YOLO::init(string cfgfile, string weightfile) {
        net = parse_network_cfg((char*)cfgfile.c_str());
        load_weights(&net, (char*)weightfile.c_str());
        set_batch_network(&net, 1);
//        srand(2222222);
        curObjId = 0;
    }

    std::vector <Object> YOLO::run_yolo(cv::Mat im0, char* camID) { // TODO use a reference of existing_obj, instead of reutrn value
        // cout << "In Run YOLO:...\n";
        struct obj_box *obj_bb_box_array;
        // vector<obj_box> obj_bb_box;
        std::vector <Object> existing_obj;
        int im_size = 448;
        char input[50];
        sprintf(input, "./data/first_frame_%s.jpg", camID);
        imwrite(input, im0);
        image im_yolo = load_image_color(input, 0, 0);
        image sized = resize_image(im_yolo, im_size, im_size);
        float *X = sized.data;
        float *predictions = network_predict(net, X);
        char str[50] = "predictions";
        obj_bb_box_array = draw_detection(im_yolo, predictions, 7, str);
        // cout << "YOLO returned\n";
        // while (obj_bb_box_array[idx].left !=YOLO_MARK){
        if (obj_bb_box_array != NULL) {
            for (int idx = 0; idx < 50 && (obj_bb_box_array[idx].left != YOLO_MARK); idx++) {
                Object tmp_obj;
                // obj_box tmp_box = obj_bb_box_array[idx];
                // cout << "test\n";
                tmp_obj.box = obj_bb_box_array[idx];
                // cout << "test2\n";
                // obj_bb_box.push_back(tmp_box);
                int x = (tmp_obj.box.left + tmp_obj.box.right) / 2;
                int y = (tmp_obj.box.top + tmp_obj.box.bot) / 2;
                double area = (tmp_obj.box.right - tmp_obj.box.left) * (tmp_obj.box.bot - tmp_obj.box.top);
                if (DEBUG)
                    printf("YOLO returned: %f %s [left,top,right,bot]:%d %d %d %d, centroid: %d,%d, area: %f\n",
                           tmp_obj.box.score, tmp_obj.box.type, tmp_obj.box.left, tmp_obj.box.top, tmp_obj.box.right,
                           tmp_obj.box.bot, x, y, area);

                if ((BOX_AREA_FILTER && area > BOX_AREA_THRESH) || !BOX_AREA_FILTER) {

                    existing_obj.push_back(tmp_obj);
                }
                // printf("Pushed back: %f %s [left,top,right,bot]:%d %d %d %d, centroid: %d,%d\n", existing_obj[idx].box.score, existing_obj[idx].box.type,existing_obj[idx].box.left,existing_obj[idx].box.top,existing_obj[idx].box.right,existing_obj[idx].box.bot, x,y);
                // idx++;
            }
//            std::sort(existing_obj.begin(), existing_obj.end(), compareByScore);//TODO: uncommet
        }
        free_image(im_yolo);
        free_image(sized);
        // cout << "test\n";

        // printf("YOLO returned: found %d objects, existing_obj[0].box.left %d\n", int(existing_obj.size()),existing_obj[0].box.left);
        ///////////////////////////////// object && CMT vector initialization///////////////////////////////////
        for (int idx = 0;idx<int(existing_obj.size()) && (existing_obj[idx].box.left!=YOLO_MARK);idx++){
            if (DEBUG) printf("Pushed back: %f %s [left,top,right,bot]:%d %d %d %d\n", existing_obj[idx].box.score, existing_obj[idx].box.type,existing_obj[idx].box.left,existing_obj[idx].box.top,existing_obj[idx].box.right,existing_obj[idx].box.bot);
            curObjId++;
//            std::cout << "Object No. " << curObjId << "\n";
            existing_obj[idx].id = curObjId;
            existing_obj[idx].alive = true;
            existing_obj[idx].tracked = true;
            existing_obj[idx].status = UNKNOWN;
            existing_obj[idx].view = UNKNOWN;
            existing_obj[idx].direction = 0;
            existing_obj[idx].displacement = 0;
            existing_obj[idx].evict_count = 0;
            existing_obj[idx].move_count = 0;
            existing_obj[idx].last_box = existing_obj[idx].box;

//            if (USE_CMT){
//
//                //creating the rectangle for bb_box
//                Rect rect_tmp = Rect(existing_obj[idx].box.left,existing_obj[idx].box.top,existing_obj[idx].box.right-existing_obj[idx].box.left,existing_obj[idx].box.bot-existing_obj[idx].box.top);
//                //legalize the bbox
//                rect_tmp = legalize_box(im0,rect_tmp);
//                ////////////////////////////// dynamically create cmt objs for each bounding box////////////////////////////
//                CMT cmt_tmp;
//                cmt_tmp.initialize(curGray, rect_tmp, im0);
//                cmt_vec.push_back(cmt_tmp); // THE ID IS THE INDEX, MATCHED WITH THE INDEX OF OBJ_BB_BOX
//                FILE_LOG(logINFO) << "Using " << rect_tmp.x << "," << rect_tmp.y << "," << rect_tmp.x + rect_tmp.width << "," << rect_tmp.y+rect_tmp.height
//                                  << "for objs #" << existing_obj[idx].id << " as initial bounding box.";
//            }
        }

        return existing_obj;
    }

YOLO::YOLO() {}

YOLO::~YOLO() {

}
