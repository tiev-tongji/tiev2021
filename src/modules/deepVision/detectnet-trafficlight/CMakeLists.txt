
file(GLOB detectnetConsoleSources *.cpp)
file(GLOB detectnetConsoleIncludes *.h )

cuda_add_executable(detectnet-trafficlight ${detectnetConsoleSources})
find_package( OpenCV REQUIRED )
include_directories(/opt/pylon5/include)
link_directories(/opt/pylon5/lib64)
target_link_libraries(detectnet-trafficlight ${OpenCV_LIBS} zcm pylonutility pylonbase GCBase_gcc_v3_1_Basler_pylon_v5_1)
target_link_libraries(detectnet-trafficlight nvcaffe_parser nvinfer jetson-inference)
