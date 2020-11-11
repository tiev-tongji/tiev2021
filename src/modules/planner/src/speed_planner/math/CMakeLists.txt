cmake_minimum_required (VERSION 2.6)

include_directories(../common)

set(math_src ${math_src} linear_interpolation.cpp)
set(math_src ${math_src} path_matcher.cpp)
set(math_src ${math_src} box.cpp)

add_library(math ${math_src})
