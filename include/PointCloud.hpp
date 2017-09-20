// PointCloud.hpp
#ifndef __POINT_CLOUD_INCLUDE__
#define __POINT_CLOUD_INCLUDE__

#include <zed/utils/GlobalDefine.hpp>
#include <zed/Mat.hpp>

#include "utils.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <mutex>

#include "Shader.hpp"

class PointCloud {
public:
    PointCloud(unsigned int width, unsigned int height,CUcontext ctx);
    ~PointCloud();

	/*
	  Initialize Opengl and Cuda buffers.
	  Warning!: must be called in the Opengl thread
	*/
    void initialize();
	/*
	  Push a new PointCloud.
	  Warning!: can be called from any thread but previously, the mutex "mutexData" must be locked
	*/
    void pushNewPC(const sl::zed::Mat matXYZRGBA);
	/*
	  Update the Opengl buffer
	  Warning!: must be called in the Opengl thread
	*/
    void pushNewPC_HostToDevice(const sl::zed::Mat matXYZRGBA);
	/*
	  Update the Opengl buffer
	  Warning!: must be called in the Opengl thread
	*/
    void update();
	/*
	  Draw the PointCloud
	  Warning!: must be called in the Opengl thread
	*/
    void draw(const Eigen::Matrix4f& vp);

    /*Close PointCloud (disable update)
     *
     */
    void close();


    unsigned int getWidth();
    unsigned int getHeight();

    std::mutex mutexData;
    CUcontext cuda_zed_ctx;
private:
    unsigned int width_;
    unsigned int height_;

    sl::zed::Mat matGPU_;
    bool hasNewPCL_;
    bool initialized_;
    Shader* shader_;
    GLuint shMVPMatrixLoc_;
    size_t numBytes_;
    float* xyzrgbaMappedBuf_;
    GLuint bufferGLID_;
    cudaGraphicsResource* bufferCudaID_;
};
#endif /* __POINT_CLOUD_INCLUDE__ */
