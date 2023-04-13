cmake_minimum_required(VERSION 3.10)
project(huace_driver)
add_subdirectory(lib/common)
set(CMAKE_CXX_STANDARD 11)
include_directories(lib/common)
include_directories(lib/common/nature)
include_directories(lib/common/coordinate_converter)
link_directories(lib)
link_directories(lib/common)


add_executable(huace_driver main.cpp gps_processing.cpp )
target_link_libraries(huace_driver serial zcm 	coordinate_converter nature)

target_include_directories(huace_driver PUBLIC 
    ./
    ${CMAKE_SOURCE_DIR}/../../msg
)