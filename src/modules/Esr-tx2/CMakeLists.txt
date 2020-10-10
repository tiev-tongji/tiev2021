cmake_minimum_required(VERSION 2.8)
project(ESR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set( CMAKE_BUILD_TYPE "Release")

find_package(Threads REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable( Esr
        src/esr.cpp
        )

target_link_libraries(Esr 
        /usr/local/lib/libzcm.so    
        ${CMAKE_THREAD_LIBS_INIT}
        )
