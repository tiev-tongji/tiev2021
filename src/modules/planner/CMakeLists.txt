cmake_minimum_required(VERSION 2.8)
project(planner)

#set(CMAKE_BUILD_TYPE "Debug")
 set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_COMPILER g++)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -o2")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../../msg
    ${PROJECT_SOURCE_DIR}/include/decision
    ${PROJECT_SOURCE_DIR}/include/map_manager
    ${PROJECT_SOURCE_DIR}/include/message_manager
    ${PROJECT_SOURCE_DIR}/include/planner_common
    ${PROJECT_SOURCE_DIR}/include/path_planner
    ${PROJECT_SOURCE_DIR}/include/speed_planner
    ${PROJECT_SOURCE_DIR}/include/visualization_msg
    ${PROJECT_SOURCE_DIR}/include/routing
    ${PROJECT_SOURCE_DIR}/third_party
)

add_subdirectory(third_party/coordinate_converter)
add_subdirectory(src)
