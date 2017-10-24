// Hang.
// This file is for extern call from cpp program.
#ifndef DETECTION_H
#define DETECTION_H
#include "network.h"
#include "detection_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"


// int* draw_detection(image im, float *box, int side, char *label);
struct obj_box* draw_detection(image im, float *box, int side, char *label);
void train_detection(char *cfgfile, char *weightfile);
void convert_detections(float *predictions, int classes, int objectness, int background, int num_boxes, int w, int h, float thresh, float **probs, box *boxes);
void do_nms(box *boxes, float **probs, int num_boxes, int classes, float thresh);
void print_detections(FILE **fps, char *id, box *boxes, float **probs, int num_boxes, int classes, int w, int h);
void validate_detection(char *cfgfile, char *weightfile);
void test_detection(char *cfgfile, char *weightfile, char *filename);
void run_detection(int argc, char **argv);
void run_video_detection(int argc, char **argv);

struct obj_box{
	int left;
	int top;
	int right;
	int bot;
	double score;
	char* type;
};

#endif

