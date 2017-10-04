SET(execName AugmentedVR)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

#set(OpenCV_DIR /home/nsl/opencv-3.1.0/release)
find_package(OpenCV REQUIRED)
find_package(ZED REQUIRED)
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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -march=native")
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



