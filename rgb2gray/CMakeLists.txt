cmake_minimum_required(VERSION 2.6)
project(rgb2gray)

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


add_executable(rgb2gray main.cpp)

target_link_libraries( rgb2gray ${OpenCV_LIBS} )

install(TARGETS rgb2gray RUNTIME DESTINATION bin)
