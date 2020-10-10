cmake_minimum_required(VERSION 3.5)

project(multibeamlaser)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( CMAKE_BUILD_TYPE "Release")

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

set(OpenCV_DIR /usr/local/share/OpenCV/)
find_package( OpenCV REQUIRED ) #opencv_highgui opencv_imgproc

find_package(Threads REQUIRED)
find_package(PythonLibs  REQUIRED)
#find_package(Eigen3)
set(EIGEN3_INCLUDE_DIR "/home/autolab/tiev2019/src/common/Eigen")

#set(PYTHON_INCLUDE_DIRS &{PYTHON_INCLUDE_DIRS} /home/autolab/anaconda3/lib/python3.7/site-packages/numpy/core/include /home/autolab/anaconda3/include/python3.7m)

set(PYTHON_INCLUDE_DIRS &{PYTHON_INCLUDE_DIRS} /home/autolab/.conda/envs/second/lib/python3.7/site-packages/numpy/core/include /home/autolab/.conda/envs/second/include/python3.7m) 

#set(PYTHON_INCLUDE_DIRS &{PYTHON_INCLUDE_DIRS} /home/autolab/anaconda3/lib/python3.6/site-packages/numpy/core/include /home/autolab/anaconda3/include/python3.6m) 
#${EIGEN3_INCLUDE_DIR}
include_directories(include ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(include/global include/perception include/tiny ../../ ../../common)  

set(GLOBAL src/global)
set(TINY src/tiny)
set(PERCEPTION src/perception)

# OpenMP
#FIND_PACKAGE(OpenMP)
#IF(OPENMP_FOUND)
#  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#  MESSAGE(STATUS "Found OpenMP")
#ENDIF()

set(GLOBAL_FILES ${GLOBAL}/global.cpp ${GLOBAL}/pose.cpp ${GLOBAL}/transform.cpp ${GLOBAL}/grid.cpp ${GLOBAL}/veloconfig.cpp)

set(TINY_FILES  ${TINY}/tinystr.cpp ${TINY}/tinyxml.cpp ${TINY}/tinyxmlerror.cpp ${TINY}/tinyxmlparser.cpp)

set(PERCEPTION_FILES ${PERCEPTION}/appl_history.cc ${PERCEPTION}/utils.cc ${PERCEPTION}/velodyne_rings.cpp ${PERCEPTION}/velodyne.cc ${PERCEPTION}/obstacle.cpp ${PERCEPTION}/tracked_obstacle.cpp ${PERCEPTION}/linear_kalman_filter.cpp ${PERCEPTION}/successive_shortest_path.cpp ${PERCEPTION}/data_association.cpp ${PERCEPTION}/kalman_multitracker.cpp ${PERCEPTION}/box.cc  ${PERCEPTION}/integration.cc ${PERCEPTION}/second_receiver.cpp ${PERCEPTION}/MultiBeamLaser.cpp )

add_executable(MultibeamLaser ${GLOBAL_FILES} ${TINY_FILES} ${PERCEPTION_FILES} ../../common/nature/timestamp.cpp)

target_link_libraries(MultibeamLaser
		/home/autolab/.conda/envs/second/lib/libpython3.7m.so 
		#/home/autolab/anaconda3/envs/second/lib/python3.7/site-packages/cv2/cv2.cpython-37m-x86_64-linux-gnu.so
		/usr/lib/x86_64-linux-gnu/libtiff.so
		/usr/local/lib/libzcm.so 
		${CMAKE_THREAD_LIBS_INIT} 
		${Boost_LIBRARIES} 
		${OpenCV_LIBS}
)

