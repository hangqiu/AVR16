#include "CMT.h"
#include <algorithm>  
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>




//
//void getBinMask( const Mat& comMask, Mat& binMask )
//{
//    if( comMask.empty() || comMask.type()!=CV_8UC1 )
//        // CV_Error( Error::StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
//        printf("comMask is empty or has incorrect type (not CV_8UC1)\n");
//    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
//        binMask.create( comMask.size(), CV_8UC1 );
//    binMask = comMask & 1;
//}
//
//Rect zoom_rect(Rect rect, double scale){
//    int new_x, new_y, new_width, new_height;
//    new_x = rect.x + rect.height / 2 - int(double(rect.height) * scale / 2);
//    new_y = rect.y + rect.width / 2 - int(double(rect.width) * scale / 2);
//    new_width = int(double(rect.width)*scale);
//    new_height = int(double(rect.height)*scale);
//    printf("old:%d,%d,%d,%d\n",rect.x,rect.y,rect.width,rect.height );
//    printf("new:%d,%d,%d,%d\n",new_x,new_y,new_width,new_height );
//    Rect new_rect = Rect(new_x,new_y,new_width,new_height);
//    return new_rect;
//}




namespace cmt {

void CMT::initialize(const Mat im_gray, const Rect rect, const Mat im_color)
{
    FILE_LOG(logDEBUG) << "CMT::initialize() call";

    //Remember initial size
    size_initial = rect.size();

    //Remember initial image
    im_prev = im_gray;

    //Compute center of rect
    Point2f center = Point2f(rect.x + rect.width/2.0, rect.y + rect.height/2.0);

    //Initialize detector and descriptor
#if CV_MAJOR_VERSION > 2
    detector = cv::FastFeatureDetector::create();
    descriptor = cv::BRISK::create();
#else
    detector = FeatureDetector::create(str_detector);
    descriptor = DescriptorExtractor::create(str_descriptor);
#endif

    //Get initial keypoints in whole image and compute their descriptors
    vector<KeyPoint> keypoints;
    detector->detect(im_gray, keypoints);

    //Divide keypoints into foreground and background keypoints according to selection
    vector<KeyPoint> keypoints_fg;
    vector<KeyPoint> keypoints_bg;


    // //add grab cut to filter out keypoints that don't belong to object
    // Mat mask,bgdModel,fgdModel; 
    // // mask.create( im_gray->size(), cv::CV_8UC1);
    // double rect_scale = 1;//scale
    // Rect rect4GrabCut = zoom_rect(rect,rect_scale);
    // cv::grabCut( im_color, mask, rect4GrabCut, bgdModel, fgdModel, 1, cv::GC_INIT_WITH_RECT);
    
    // Mat res, binMask;
    // getBinMask( mask, binMask );
    // im_color.copyTo( res, binMask );
    // char name[128];
    // sprintf(name,"masked_pic_%d.jpg",rect.x);
    // imshow(name,res);
    // // imwrite(name,res);
    // // std::cout << name <<"\n"<< binMask << "\n";
    // // std::cout << "binmask dimension"<<binMask.dims << "rows"<<binMask.rows<<"cols"<<binMask.cols;
    // // std::cout << binMask.at<int>(1,1)<<"\n";

    //filter foreground background
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        KeyPoint k = keypoints[i];
        Point2f pt = k.pt;

        // if (pt.x > rect.x && pt.y > rect.y && pt.x < rect.br().x && pt.y < rect.br().y && binMask.at<int>(pt) != 0)//added a mask constraint
        if (pt.x > rect.x && pt.y > rect.y && pt.x < rect.br().x && pt.y < rect.br().y )
        {
            // std::cout << "("<< pt.x << "," <<pt.y <<"):"<< binMask.at<int>(pt)<<"\n";
            keypoints_fg.push_back(k);
        }

        else
        {
            keypoints_bg.push_back(k);
        }

    }

    //Create foreground classes
    vector<int> classes_fg;
    classes_fg.reserve(keypoints_fg.size());
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        classes_fg.push_back(i);
    }

    //Compute foreground/background features
    Mat descs_fg;
    Mat descs_bg;
    descriptor->compute(im_gray, keypoints_fg, descs_fg);
    descriptor->compute(im_gray, keypoints_bg, descs_bg);

    //Only now is the right time to convert keypoints to points, as compute() might remove some keypoints
    vector<Point2f> points_fg;
    vector<Point2f> points_bg;
    // hang
    // int max_num_kpoints = 30;


    // for (size_t i = 0; i < std::min((int)keypoints_fg.size(),max_num_kpoints); i++)
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        points_fg.push_back(keypoints_fg[i].pt);
    }

    FILE_LOG(logDEBUG) << points_fg.size() << " foreground points.";

    for (size_t i = 0; i < keypoints_bg.size(); i++)
    {
        points_bg.push_back(keypoints_bg[i].pt);
    }

    //Create normalized points
    vector<Point2f> points_normalized;
    for (size_t i = 0; i < points_fg.size(); i++)
    {
        points_normalized.push_back(points_fg[i] - center);
    }

    //Initialize matcher
    matcher.initialize(points_normalized, descs_fg, classes_fg, descs_bg, center);

    //Initialize consensus
    consensus.initialize(points_normalized);

    //Create initial set of active keypoints
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        points_active.push_back(keypoints_fg[i].pt);
        classes_active = classes_fg;
    }

    FILE_LOG(logDEBUG) << "CMT::initialize() return";
}

void CMT::processFrame(Mat im_gray) {

    FILE_LOG(logDEBUG) << "CMT::processFrame() call";

    //Track keypoints
    vector<Point2f> points_tracked;
    vector<unsigned char> status;
    tracker.track(im_prev, im_gray, points_active, points_tracked, status);

    FILE_LOG(logDEBUG) << points_tracked.size() << " tracked points.";

    //keep only successful classes
    vector<int> classes_tracked;
    for (size_t i = 0; i < classes_active.size(); i++)
    {
        if (status[i])
        {
            classes_tracked.push_back(classes_active[i]);
        }

    }

    //Detect keypoints, compute descriptors
    vector<KeyPoint> keypoints;
    detector->detect(im_gray, keypoints);

    FILE_LOG(logDEBUG) << keypoints.size() << " keypoints found.";

    Mat descriptors;
    descriptor->compute(im_gray, keypoints, descriptors);

    //Match keypoints globally
    vector<Point2f> points_matched_global;
    vector<int> classes_matched_global;
    matcher.matchGlobal(keypoints, descriptors, points_matched_global, classes_matched_global);

    FILE_LOG(logDEBUG) << points_matched_global.size() << " points matched globally.";

    //Fuse tracked and globally matched points
    vector<Point2f> points_fused;
    vector<int> classes_fused;
    fusion.preferFirst(points_tracked, classes_tracked, points_matched_global, classes_matched_global,
            points_fused, classes_fused);

    FILE_LOG(logDEBUG) << points_fused.size() << " points fused.";

    //Estimate scale and rotation from the fused points
    float scale;
    float rotation;
    consensus.estimateScaleRotation(points_fused, classes_fused, scale, rotation);

    FILE_LOG(logDEBUG) << "scale " << scale << ", " << "rotation " << rotation;

    //Find inliers and the center of their votes
    Point2f center;
    vector<Point2f> points_inlier;
    vector<int> classes_inlier;
    consensus.findConsensus(points_fused, classes_fused, scale, rotation,
            center, points_inlier, classes_inlier);

    FILE_LOG(logDEBUG) << points_inlier.size() << " inlier points.";
    FILE_LOG(logDEBUG) << "center " << center;

    //Match keypoints locally
    vector<Point2f> points_matched_local;
    vector<int> classes_matched_local;
    matcher.matchLocal(keypoints, descriptors, center, scale, rotation, points_matched_local, classes_matched_local);

    FILE_LOG(logDEBUG) << points_matched_local.size() << " points matched locally.";

    //Clear active points
    points_active.clear();
    classes_active.clear();

    //Fuse locally matched points and inliers
    fusion.preferFirst(points_matched_local, classes_matched_local, points_inlier, classes_inlier, points_active, classes_active);
//    points_active = points_fused;
//    classes_active = classes_fused;

    FILE_LOG(logDEBUG) << points_active.size() << " final fused points.";

    //TODO: Use theta to suppress result
    bb_rot = RotatedRect(center,  size_initial * scale, rotation/CV_PI * 180);

    //Remember current image
    im_prev = im_gray;

    FILE_LOG(logDEBUG) << "CMT::processFrame() return";
}

} /* namespace CMT */
