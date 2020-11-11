find_package(OpenCV 3.3.0 EXACT)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(visualization main.cpp visualization.cpp)
target_link_libraries(visualization planner_common speed_planner zcm ${OpenCV_LIBS})
