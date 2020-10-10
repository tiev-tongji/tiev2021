cmake_minimum_required(VERSION 3.5)
project(IMU_Receiver)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)

#ZCM
set(ZCM_INCLUDE_DIR /usr/local/include/zcm)
set(ZCM_LIBRARY_DIR /usr/local/lib)

#include
include_directories(../../)
include_directories(../../common)
include_directories(../../msg/include)
include_directories(../../common/nature)
include_directories(../../common/coodinate_converter)

#lib
link_directories(ZCM_LIBRARY_DIR)

set(SOURCE_FILES receiver.cpp NComRxC.h NComRxC.c 
		../../common/nature/timestamp.cpp
		../../common/nature/angle.cpp
		../../common/coordinate_converter/coordinate_converter.cpp
		../../common/coordinate_converter/basic_coordinate_converter.cpp


		)

add_executable(IMU_Receiver ${SOURCE_FILES})

target_link_libraries(IMU_Receiver
        #zcm
	libzcm.so
        ${CMAKE_THREAD_LIBS_INIT}
        )

