cmake_minimum_required(VERSION 2.8)
project(Lux)

set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/local/include/)
include_directories(${PROJECT_SOURCE_DIR} ../../)

#lib
set(SOURCE_FILES main.cpp lux_8.h)

add_executable(lux ${SOURCE_FILES} ../../common/nature/timestamp.cpp)

find_package(Threads REQUIRED)

if(THREADS_HAVE_PTHREAD_ARG)
    set_property(TARGET perception PROPERTY COMPILE_OPTIONS "-pthread")
    set_property(TARGET perception PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

target_link_libraries(lux
        /usr/local/lib/libzcm.so 
        ${CMAKE_THREAD_LIBS_INIT}
        )
