cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( CMAKE_BUILD_TYPE "Debug" )

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

find_package(OpenCV REQUIRED objdetect opencv_highgui )#opencv_highgui opencv_imgproc

find_package(Threads REQUIRED)

#find_package(Eigen3)
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(../../../common/Eigen)

include_directories(include ${Boost_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
include_directories( "/usr/include/lua5.2/" include/)
include_directories(../../../)
include_directories(../../../msg/include)
include_directories(../../../common/)

include_directories(../Cartographer/)
include_directories(../Cartographer/build/cartographer/)
include_directories(../Cartographer/build)

link_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/../Cartographer/build)

set(OpenCV_DIR /usr/local/share/OpenCV/)
find_package(OpenCV REQUIRED)

find_package(Protobuf REQUIRED)
message("PROTOBUF_INCLUDE_DIR: " ${PROTOBUF_INCLUDE_DIR})
message("PROTOBUF_LIBRARY: " ${PROTOBUF_LIBRARY})
message("PROTOBUF_PROTOC_EXECUTABLE: " ${PROTOBUF_PROTOC_EXECUTABLE})

set(SOURCE_FILES
        msg.cpp
        node.cpp
        map_builder_bridge.cpp
        node_option.cpp
        trajectory_option.cpp
        #tf_eigen.cpp
		image_fusion.cpp
		#test.cpp
	    virtualscan.cpp
		../../../common/nature/timestamp.cpp
		map_offset.cpp
        ../../../common/coordinate_converter/coordinate_converter.cpp
        ../../../common/nature/angle.cpp
        ../../../common/coordinate_converter/basic_coordinate_converter.cpp
		)

add_executable(perceptionfusion main.cpp ${SOURCE_FILES})
add_executable(perceptionfusion_for_competition main_competition.cpp ${SOURCE_FILES})

set(LINK_LIBS
		${OpenCV_LIBS}
		libcartographer.so
		/usr/local/lib/libceres.a
		${Boost_LIBRARIES}
		boost_iostreams
		pthread
		zcm
		glog
		${PROTOBUF_LIBRARY})

target_link_libraries(perceptionfusion
		${LINK_LIBS})
target_link_libraries(perceptionfusion_for_competition
		${LINK_LIBS})
