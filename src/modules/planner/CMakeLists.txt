cmake_minimum_required(VERSION 3.10)
project(planner)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
    message("Building planner in Debug mode")
else()
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    message("Building planner in Release mode")
endif()

set(CMAKE_CXX_COMPILER g++)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/visualization_zcm
    ${PROJECT_SOURCE_DIR}/third_party
)

add_subdirectory(visualization_zcm)
add_subdirectory(third_party/coordinate_converter)
add_subdirectory(src)
