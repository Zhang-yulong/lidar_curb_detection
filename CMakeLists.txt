cmake_minimum_required(VERSION 3.0.2)
project(lidar_curb_detection)

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)

set(3rdparty /home/zyl/3rdparty/echiev/linux-x86-64)

set(OpenCV_DIR ${3rdparty}/opencv/opencv_3.3/share/OpenCV)
find_package(OpenCV REQUIRED)

# set(Eigen3_DIR "/home/zyl/3rdparty/eigen-3.3.7/cmake")
# find_package(Eigen3 REQUIRED)

find_package(VTK)
find_package(PCL 1.8 COMPONENTS common visualization io surface segmentation REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


INCLUDE_DIRECTORIES(
   
    # ${3rdparty}/ls_driver/Include
    # ${3rdparty}/comm_2.7.1/include
    # ${3rdparty}/libconfig/include
    # ${3rdparty}/ulog/include
    ${3rdparty}/comm/comm_2.6.2/include
    ${3rdparty}/libconfig/include
    ${3rdparty}/ulog/include
    ${3rdparty}/eigen/include/eigen3
    ${3rdparty}/opencv/opencv_3.3/include
    ${3rdparty}/opencv/opencv_3.3/include/opencv
    ${3rdparty}/opencv/opencv_3.3/include/opencv2

    ./include
    ./common
    ./ls_driver/Include
    # ${EIGEN3_INCLUDE_DIR}
)

# LINK_DIRECTORIES("${3rdparty}/comm_2.7.1/lib")
LINK_DIRECTORIES("${3rdparty}/comm/comm_2.6.2/lib")
LINK_DIRECTORIES("${3rdparty}/libconfig/lib")
LINK_DIRECTORIES("${3rdparty}/ulog/lib")
LINK_DIRECTORIES("${3rdparty}/opencv/opencv_3.3/lib")
LINK_LIBRARIES("${3rdparty}/opencv/opencv_3.3/lib/libopencv_core.so" )

# AUX_SOURCE_DIRECTORY(${3rdparty}/ls_driver/Include LS)
AUX_SOURCE_DIRECTORY(./ls_driver/Include LS)
AUX_SOURCE_DIRECTORY(./common COMMON)    
AUX_SOURCE_DIRECTORY(./include SRC)

SET(COMM ${3rdparty}/comm/comm_2.6.2/lib/libcomm.so)

add_executable(lidar_curb_detection src/main.cpp ${LS} ${SRC} ${COMMON} )
target_link_libraries(lidar_curb_detection pthread ${OpenCV_LIBS} ${PCL_LIBRARIES} config ulog ${COMM} )

