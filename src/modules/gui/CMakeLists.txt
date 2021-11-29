cmake_minimum_required(VERSION 3.1.0)

project(tvis)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
    message("Building tvis in Debug mode")
else()
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    message("Building tvis in Release mode")
endif()

set(CMAKE_CXX_COMPILER g++)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

add_executable(tvis
    tviswindow.cpp
    main.cpp
    coordinate_converter.cpp
    tiev_elements.cpp
    camera.cpp
    message_manager.cpp
)

target_include_directories(tvis PUBLIC ./ ${CMAKE_SOURCE_DIR}/../../msg)

target_link_libraries(tvis Qt5::Core Qt5::Gui zcm)