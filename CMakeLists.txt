#project(Augmented_VR)
#
#CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#
#IF(NOT CMAKE_BUILD_TYPE)
#	SET(CMAKE_BUILD_TYPE Debug)
#ENDIF()
#
#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
#
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
#
## Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#	set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
#	add_definitions(-DCOMPILEDWITHC11)
#	message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#	set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
#	add_definitions(-DCOMPILEDWITHC0X)
#	message(STATUS "Using flag -std=c++0x.")
#else()
#	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()
#
#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#
#find_package(OpenCV 3.1.0 REQUIRED)
#if(NOT OpenCV_FOUND)
#	find_package(OpenCV 2.4.3 QUIET)
#	if(NOT OpenCV_FOUND)
#		message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#	endif()
#endif()
#
#
#SET(EXECUTABLE_OUTPUT_PATH ".")
#SET(SPECIAL_OS_LIBS "")
#SET(VERSION_REQ_CUDA "8.0")
#
#
#find_package(CUDA QUIET REQUIRED)
#set(CUDA_PROPAGATE_HOST_FLAGS OFF)
#SET(CUDA_HOST_COMPILER /usr/bin/g++)
#LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")
#
#
#find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(ZED 2 REQUIRED)
#find_package(GLUT REQUIRED)
#find_package(GLEW REQUIRED)
#find_package(OpenGL REQUIRED)
#find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
#find_package(PCL 1.8 REQUIRED)
#find_package(OpenGL REQUIRED)
#find_package(OpenSSL)
#find_package(Threads)
#
#cuda_include_directories(
#		${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
#)
#
#include_directories(
#		${PROJECT_SOURCE_DIR}
#		${PROJECT_SOURCE_DIR}/src/ORB_SLAM2
#		${PROJECT_SOURCE_DIR}/src/ORB_SLAM2/include
#		${PROJECT_SOURCE_DIR}/include
#		${EIGEN3_INCLUDE_DIR}
#		${Pangolin_INCLUDE_DIRS}
#		${ZED_INCLUDE_DIRS}
#		${GLEW_INCLUDE_DIRS}
#		${GLUT_INCLUDE_DIRS}
#		${CUDA_INCLUDE_DIRS}
#		${PCL_INCLUDE_DIRS}
#)
#
#
#link_directories(${ZED_LIBRARY_DIR})
#link_directories(${GLEW_LIBRARY_DIR})
#link_directories(${GLUT_LIBRARY_DIRS})
#link_directories(${OpenGL_LIBRARY_DIRS})
#link_directories(${CUDA_LIBRARY_DIRS})
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
#link_directories(${PCL_LIBRARY_DIRS})
#
#
##message(${PCL_LIBRARY_DIRS})
##message(${CUDA_INCLUDE_DIRS})
#
##SET(SRC_FOLDER src)
##FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/AugmentedVR/*.cpp" "${SRC_FOLDER}/ORB_SLAM2/src/*.cc" "${SRC_FOLDER}/*.cpp")
##FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/AugmentedVR/*.cpp" "${SRC_FOLDER}/ORB_SLAM2/src/*.cc")
#
##message(${SRC_FILES})
#
##SET(HEADER_FOLDER include)
##FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h" "${HEADER_FOLDER}/../src/ORB_SLAM2/include/*.h")
#
##message(${HEADER_FILES})
##set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "ZED Multi Input")
#
##add_definitions(-std=c++0x)
##add_subdirectory(src/ORB_SLAM2)
#
##message("${CUDA_LIBRARIES}")
##message("${CUDA_nppi_LIBRARY}")
##message("${CUDA_npps_LIBRARY}")
##message("${CUDA_CUDA_LIBRARIES}")
##message("${CUDA_npp_LIBRARY}")
##message("${CUDA_CUDART_LIBRARY}")
#
##SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" ) # Release Perf mode
##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g -march=native")
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
##
#
#
#cuda_add_library(${PROJECT_NAME} SHARED
#        #${ORB_SLAM2_SRC}
#        #${HDR_FILES}
#        #${SRC_FILES}
#        src/ORB_SLAM2/src/System.cc
#        src/ORB_SLAM2/src/Tracking.cc
#        src/ORB_SLAM2/src/LocalMapping.cc
#        src/ORB_SLAM2/src/LoopClosing.cc
#        src/ORB_SLAM2/src/ORBextractor.cc
#        src/ORB_SLAM2/src/ORBmatcher.cc
#        src/ORB_SLAM2/src/FrameDrawer.cc
#        src/ORB_SLAM2/src/Converter.cc
#        src/ORB_SLAM2/src/MapPoint.cc
#        src/ORB_SLAM2/src/KeyFrame.cc
#        src/ORB_SLAM2/src/Map.cc
#        src/ORB_SLAM2/src/MapDrawer.cc
#        src/ORB_SLAM2/src/Optimizer.cc
#        src/ORB_SLAM2/src/PnPsolver.cc
#        src/ORB_SLAM2/src/Frame.cc
#        src/ORB_SLAM2/src/KeyFrameDatabase.cc
#        src/ORB_SLAM2/src/Sim3Solver.cc
#        src/ORB_SLAM2/src/Initializer.cc
#        src/ORB_SLAM2/src/Viewer.cc
#        app/ZEDInitialization.cpp
#        #src/GLViewer.cpp
#        )
#
#
#
#
##ADD_EXECUTABLE(${execName} ${SRC_FILES} ./app/main.cpp src/AugmentedVR/Global.cpp include/Global.hpp src/AugmentedVR/FrameCache.cpp include/FrameCache.hpp src/AugmentedVR/AVRFrame.cpp include/AVRFrame.hpp src/AugmentedVR/CVUtils.cpp include/CVUtils.hpp src/AugmentedVR/PCUtils.cpp include/PCUtils.hpp)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME}
#
#
#        ${ZED_LIBRARIES}
#        ${OPENGL_LIBRARIES}
#        ${GLUT_LIBRARY}
#        ${GLEW_LIBRARY}
#        ${OpenCV_LIBRARIES}
#		${PCL_LIBRARIES}
#		${CUDA_LIBRARY} ${CUDA_CUDA_LIBRARIES} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
#		${Pangolin_LIBRARIES}
#		#		src/ORB_SLAM2/lib/libORB_SLAM2.so
#		${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#		${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#		${Boost_LIBRARIES}
#		/usr/lib/x86_64-linux-gnu/libcuda.so.1
#		nvToolsExt
#		)
#
#
#
#add_executable(main_cpp app/mainScript.cpp ${SRC_FILES} ../app/ZEDInitialization.cpp ../include/ZEDInitialization.h)
#
#TARGET_LINK_LIBRARIES(main_cpp
#       ${PROJECT_NAME}
#                )
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
##
##
##cmake_minimum_required(VERSION 2.8)
##project(Crowdsourced_HD_Map)
##
##IF(NOT CMAKE_BUILD_TYPE)
##    SET(CMAKE_BUILD_TYPE Debug)
##ENDIF()
##
##MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
##
##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
##
### Check C++11 or C++0x support
##include(CheckCXXCompilerFlag)
##CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
##CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
##if(COMPILER_SUPPORTS_CXX11)
##    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
##    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
##    add_definitions(-DCOMPILEDWITHC11)
##    message(STATUS "Using flag -std=c++11.")
##elseif(COMPILER_SUPPORTS_CXX0X)
##    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
##    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
##    add_definitions(-DCOMPILEDWITHC0X)
##    message(STATUS "Using flag -std=c++0x.")
##else()
##    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
##endif()
##
##LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
##
##find_package(OpenCV 3.1.0 REQUIRED)
##if(NOT OpenCV_FOUND)
##    find_package(OpenCV 2.4.3 QUIET)
##    if(NOT OpenCV_FOUND)
##        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
##    endif()
##endif()
##
##SET(EXECUTABLE_OUTPUT_PATH ".")
##SET(SPECIAL_OS_LIBS "")
##SET(VERSION_REQ_CUDA "8.0")
##
##
##find_package(CUDA QUIET REQUIRED)
##set(CUDA_PROPAGATE_HOST_FLAGS OFF)
##SET(CUDA_HOST_COMPILER /usr/bin/g++)
##LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")
##
##
##find_package(Eigen3 REQUIRED)
##find_package(Pangolin REQUIRED)
##find_package(ZED 2 REQUIRED)
##find_package(GLUT REQUIRED)
##find_package(GLEW REQUIRED)
##find_package(OpenGL REQUIRED)
##find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
##find_package(PCL 1.8 REQUIRED)
##
##cuda_include_directories(
##        ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
##)
##
##
##include_directories(
##        ${PROJECT_SOURCE_DIR}
##        ${PROJECT_SOURCE_DIR}/ORB_SLAM2
##        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/include
##        ${PROJECT_SOURCE_DIR}/include
##        ${EIGEN3_INCLUDE_DIR}
##        ${Pangolin_INCLUDE_DIRS}
##        ${ZED_INCLUDE_DIRS}
##        ${GLEW_INCLUDE_DIRS}
##        ${GLUT_INCLUDE_DIRS}
##        ${CUDA_INCLUDE_DIRS}
##        ${PCL_INCLUDE_DIRS}
##)
##
##
##link_directories(${ZED_LIBRARY_DIR})
##link_directories(${GLEW_LIBRARY_DIRS})
##link_directories(${GLUT_LIBRARY_DIRS})
##link_directories(${OpenGL_LIBRARY_DIRS})
##link_directories(${CUDA_LIBRARY_DIRS})
##link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
##link_directories(${PCL_LIBRARY_DIRS})
##
##FILE(GLOB_RECURSE SRC_FILES src/ORB_SLAM2/src/ *.cpp)
##FILE(GLOB_RECURSE HDR_FILES src/ORB_SLAM2/include/ *.h)
##FILE(GLOB_RECURSE ORB_SLAM2_SRC src/ORB_SLAM2/src/*.cc)
##
##
##set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
##
##cuda_add_library(${PROJECT_NAME} SHARED
##        #${ORB_SLAM2_SRC}
##        #${HDR_FILES}
##        #${SRC_FILES}
##        src/ORB_SLAM2/src/System.cc
##        src/ORB_SLAM2/src/Tracking.cc
##        src/ORB_SLAM2/src/LocalMapping.cc
##        src/ORB_SLAM2/src/LoopClosing.cc
##        src/ORB_SLAM2/src/ORBextractor.cc
##        src/ORB_SLAM2/src/ORBmatcher.cc
##        src/ORB_SLAM2/src/FrameDrawer.cc
##        src/ORB_SLAM2/src/Converter.cc
##        src/ORB_SLAM2/src/MapPoint.cc
##        src/ORB_SLAM2/src/KeyFrame.cc
##        src/ORB_SLAM2/src/Map.cc
##        src/ORB_SLAM2/src/MapDrawer.cc
##        src/ORB_SLAM2/src/Optimizer.cc
##        src/ORB_SLAM2/src/PnPsolver.cc
##        src/ORB_SLAM2/src/Frame.cc
##        src/ORB_SLAM2/src/KeyFrameDatabase.cc
##        src/ORB_SLAM2/src/Sim3Solver.cc
##        src/ORB_SLAM2/src/Initializer.cc
##        src/ORB_SLAM2/src/Viewer.cc
##        app/ZEDInitialization.cpp
##        #src/GLViewer.cpp
##        )
##
##add_definitions(${PCL_DEFINITIONS})
##
##target_link_libraries(${PROJECT_NAME}
##        ${OpenCV_LIBS}
##        ${EIGEN3_LIBS}
##        ${Pangolin_LIBRARIES}
##        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
##        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
##        ${SPECIAL_OS_LIBS}
##        ${ZED_LIBRARIES}
##        ${OPENGL_LIBRARIES}
##        ${GLUT_LIBRARIES}
##        ${GLEW_LIBRARIES}
##        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
##        ${PCL_LIBRARIES}
##        nvToolsExt
##        )
##
##message(STATUS "Compile With map save/load function")
##find_library(BOOST_SERIALIZATION boost_serialization)
##if (NOT BOOST_SERIALIZATION)
##    message(FATAL_ERROR "Can't find libboost_serialization")
##endif()
##target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})
##
### Build examples
##
##add_executable(main_cpp
##        app/mainScript.cpp)
##target_link_libraries(main_cpp
##        ${PROJECT_NAME})
###
###add_executable(point_cloud_viewer
###        src/PointCloudViewer.cpp)
###
###target_link_libraries(point_cloud_viewer
###        ${PROJECT_NAME})
###
###add_executable(stereo_kitti
###        ORB_SLAM2/Examples/Stereo/stereo_kitti.cc)
###target_link_libraries(stereo_kitti ${PROJECT_NAME})
###
###
###add_executable(parseTrajectories
###        src/parseTrajectoryFiles.cpp)
###target_link_libraries(parseTrajectories ${PROJECT_NAME})
###
###add_executable(test
###        src/test.cpp)
###
###



SET(execName AugmentedVR)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

#set(OpenCV_DIR /home/nsl/opencv-3.1.0/release)
find_package(OpenCV 3.1.0 REQUIRED)
find_package(ZED 2.0 REQUIRED)
find_package(GLUT)
find_package(GLEW)
find_package(OpenGL)
find_package(OpenSSL)
find_package(Threads)

#LIST(APPEND CMAKE_MODULE_PATH /home/hang/AugmentedVR/src/ORB_SLAM2/cmake_modules/)
#message(${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(PCL)
find_package(Pangolin REQUIRED)
find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
        REQUIRED             					# Fail with error if Boost is not found
        COMPONENTS serialization 				# Boost libraries by their canonical name
        )



include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(src/ORB_SLAM2/include)
include_directories(Thirdparty/DBoW2/DBoW2)


link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIR})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(Thirdparty/DBoW2/lib)
link_directories(Thirdparty/g2o/lib)


message(${PCL_LIBRARY_DIRS})
#message(${CUDA_INCLUDE_DIRS})

SET(SRC_FOLDER src)
#FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/AugmentedVR/*.cpp" "${SRC_FOLDER}/ORB_SLAM2/src/*.cc" "${SRC_FOLDER}/*.cpp")
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/AugmentedVR/*.cpp" "${SRC_FOLDER}/ORB_SLAM2/src/*.cc")

#message(${SRC_FILES})

SET(HEADER_FOLDER include)
FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h" "${HEADER_FOLDER}/../src/ORB_SLAM2/include/*.h")

message(${HEADER_FILES})
#set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "ZED Multi Input")

#add_definitions(-std=c++0x)
#add_subdirectory(src/ORB_SLAM2)

message("${CUDA_LIBRARIES}")
message("${CUDA_nppi_LIBRARY}")
message("${CUDA_npps_LIBRARY}")
message("${CUDA_CUDA_LIBRARIES}")
message("${CUDA_npp_LIBRARY}")
message("${CUDA_CUDART_LIBRARY}")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" ) # Release Perf mode
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_EXECUTABLE(${execName} ${SRC_FILES} ./app/main.cpp src/AugmentedVR/Global.cpp include/Global.hpp src/AugmentedVR/FrameCache.cpp include/FrameCache.hpp src/AugmentedVR/AVRFrame.cpp include/AVRFrame.hpp src/AugmentedVR/CVUtils.cpp include/CVUtils.hpp src/AugmentedVR/PCUtils.cpp include/PCUtils.hpp)
TARGET_LINK_LIBRARIES(${execName}
        ${ZED_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        ${GLEW_LIBRARY}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CUDA_LIBRARY} ${CUDA_CUDA_LIBRARIES} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
        ${Pangolin_LIBRARIES}
        #		src/ORB_SLAM2/lib/libORB_SLAM2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${Boost_LIBRARIES}
        /usr/lib/x86_64-linux-gnu/libcuda.so.1
        )



