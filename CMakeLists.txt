SET(execName ZED_Multi_Input)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

#IF(WIN32) # Windows
##    message(FATAL_ERROR "Multi ZED not available on Windows")
##    EXIT()
#ELSE() # Linux
#    find_package(ZED 1.0 REQUIRED)
#    find_package(GLUT)
#    find_package(GLEW)
#    find_package(OpenGL)

#    SET(Eigen_INCLUDE_DIRS "/usr/include/eigen3") ##depends if installed by packet or by own build install
#    SET(Eigen_INCLUDE_DIRS_2 "/usr/local/include/eigen3") ##depends if installed by packet or by own build install
#    include_directories(${Eigen_INCLUDE_DIRS_2})

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
#     exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
#     if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
#        find_package(OpenCV 2.4)
#     else()
#      find_package(OpenCV 3.1)
#     endif()


#ENDIF(WIN32)


#set(OpenCV_DIR /home/nsl/opencv-3.1.0/release)
find_package(OpenCV REQUIRED)
find_package(ZED 1.0 REQUIRED)
find_package(GLUT)
find_package(GLEW)
find_package(OpenGL)

LIST(APPEND CMAKE_MODULE_PATH /home/hang/AugmentedVR/src/ORB_SLAM2/cmake_modules/)
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


link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIR})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${PCL_LIBRARY_DIRS})

message(${PCL_LIBRARY_DIRS})

#message(${CUDA_INCLUDE_DIRS})

SET(SRC_FOLDER src)
#FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/AugmentedVR/*.cpp" "${SRC_FOLDER}/ORB_SLAM2/src/*.cc" "${SRC_FOLDER}/*.cpp")

#message(${SRC_FILES})

SET(HEADER_FOLDER include)
#FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h")
FILE(GLOB_RECURSE HEADER_FILES "${HEADER_FOLDER}/*.hpp" "${HEADER_FOLDER}/*.h" "${HEADER_FOLDER}/../src/ORB_SLAM2/include/*.h")

#message(${HEADER_FILES})
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

ADD_EXECUTABLE(${execName} ${SRC_FILES} ./app/main.cpp)
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

#ADD_EXECUTABLE(AVR_Receiver ${SRC_FILES} ./app/Receiver.cpp)
#ADD_EXECUTABLE(AVR_Sender ${SRC_FILES} ./app/Sender.cpp)
#
#TARGET_LINK_LIBRARIES(AVR_Receiver
#		${ZED_LIBRARIES}
#		${OPENGL_LIBRARIES}
#		${GLUT_LIBRARY}
#		${GLEW_LIBRARY}
#		${OpenCV_LIBRARIES}
#		${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
#		${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY}
#		${Pangolin_LIBRARIES}
#		#		src/ORB_SLAM2/lib/libORB_SLAM2.so
#		${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#		${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#		${Boost_LIBRARIES}
#		)
#TARGET_LINK_LIBRARIES(AVR_Sender
#		${ZED_LIBRARIES}
#		${OPENGL_LIBRARIES}
#		${GLUT_LIBRARY}
#		${GLEW_LIBRARY}
#		${OpenCV_LIBRARIES}
#		${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
#		${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY}
#		${Pangolin_LIBRARIES}
#		#		src/ORB_SLAM2/lib/libORB_SLAM2.so
#		${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#		${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#		${Boost_LIBRARIES}
#		)


