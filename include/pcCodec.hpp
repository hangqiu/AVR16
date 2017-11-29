//
// Created by hang on 3/11/17.
//

#ifndef PROJECT_PCCODEC_H
#define PROJECT_PCCODEC_H
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>

#include "globals.hpp"

#include <opencv2/opencv.hpp>

//#include <zed/Camera.hpp>
//#include <zed/utils/GlobalDefine.hpp>
#include <sl/Camera.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/compression/octree_pointcloud_compression.h>


//#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
////#include <pcl/io/pcd_io.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace sl;
using namespace std;
using namespace pcl;

class pcCodec {
//    float* data_cloud;
    std::mutex mutex_input;
    int width, height;


    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;

    pcl::io::compression_Profiles_e compressionProfile;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_xyzrgba_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyz_ptr;
    int resRatio = 20;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyz_ptr_lowRes;


    pcl::ModelCoefficients::Ptr planeSeg_coefficients_ptr;
    pcl::PointIndices::Ptr planeSeg_inliers_ptr;

    bool cv2pcl_xyzrgba(cv::Mat &pc);

    bool cv2pcl_xyz(cv::Mat &pc);

    bool cv2pcl_xyz_lowRes(cv::Mat &pc);

    cv::Mat pcl2cv_xyz(cv::Mat &pc);



public:
    pcCodec(int width, int height);

    ~pcCodec();


    void encode(cv::Mat& pc);

    void planeSegmentation(cv::Mat &pc, sl::Mat & slpc);

    void planeSegmentation_ManualPlaneModel(cv::Mat &pc, sl::Mat & slpc, float A, float B, float C, float D);

//    void euclideanSegmentation(cv::Mat &pc);
};


#endif //PROJECT_PCCODEC_H
