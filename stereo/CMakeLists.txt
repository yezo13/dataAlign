cmake_minimum_required(VERSION 2.8)
project(OpticalSLAM)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(Pangolin 0.4 REQUIRED)
# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(${Pangolin_INCLUDE_DIRS} 
"./include" 
"/usr/include/eigen3")

add_executable(kitti_slam kitti_main.cpp)
target_link_libraries(kitti_slam ${Pangolin_LIBRARIES} ${OpenCV_LIBS} OpticalSLAM)

add_subdirectory(src)