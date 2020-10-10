include_directories( ${OpenCV_INCLUDE_DIRS} )
add_library(visualization visualization.cpp)
target_link_libraries(visualization ${OpenCV_LIBS})
