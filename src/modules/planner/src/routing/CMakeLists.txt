# CMake版本约束
cmake_minimum_required(VERSION 3.10)
# 工程名称
project(test_find_path)

set(CMAKE_C_FLAGS "$ {CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
SET(CMAKE_BUILD_TYPE Debug)
include_directories(
        ./
        ./../../common/
        ./../../common/nature/ )
set(SRC Routing.cpp Config.cpp)

if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
    add_subdirectory(../../common common)
endif()

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
link_directories(${LIBRARY_OUTPUT_PATH})    #lib搜索路径

add_library(routing SHARED ${SRC})
target_link_libraries(routing nature coordinate_converter pqxx -L/usr/lib/x86_64-linux-gnu)

# 增加可执行文件
# add_executable(${PROJECT_NAME} main.cpp)
# target_link_libraries(${PROJECT_NAME} librouting.so)
