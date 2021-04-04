find_package(OpenCV REQUIRED)
add_executable(visualization "")
target_sources(visualization PRIVATE
    main.cpp 
    visualization.cpp
    visualization.h
)
target_include_directories(visualization PRIVATE
    ${CMAKE_SOURCE_DIR}/include/visualization_msg
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(visualization PRIVATE 
    planner_common 
    speed_planner 
    zcm 
    ${OpenCV_LIBS}
)
