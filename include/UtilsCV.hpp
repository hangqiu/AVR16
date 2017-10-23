//
// Created by hang on 9/26/17.
//

#ifndef PROJECT_CVUTILS_H
#define PROJECT_CVUTILS_H

#include <opencv2/core/types.hpp>
#include <include/globals.hpp>

void debugCin();
void debugPauser();
void processKey(char key);
void pauseStopResume();
void saveCurFrame(cv::Mat FrameLeft, int frameSeq, long frameTS);



#endif //PROJECT_CVUTILS_H
