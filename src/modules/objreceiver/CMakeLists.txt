cmake_minimum_required(VERSION 2.8.3)
project(Objreceiver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( CMAKE_BUILD_TYPE "Release")

#find_package(Boost REQUIRED COMPONENTS system thread filesystem)

find_package(OpenCV REQUIRED)# objdetect opencv_highgui opencv_highgui opencv_imgproc

find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}) #${Boost_INCLUDE_DIRS} "/usr/include/eigen3" 
include_directories(../../ )

add_executable(objreceiver receiver.cpp)

target_link_libraries(objreceiver zcm ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS}) #${Boost_LIBRARIES}