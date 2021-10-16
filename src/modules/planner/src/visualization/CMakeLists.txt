find_package(OpenCV REQUIRED)
add_executable(visualization "")
target_sources(visualization PRIVATE
    main.cpp 
    visualization.cpp
    visualization.h
)
target_include_directories(visualization PUBLIC 
    ${CMAKE_SOURCE_DIR}/visualization_zcm
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(visualization PUBLIC 
    planner_common 
    speed_planner 
    message_manager
    zcm 
    ${OpenCV_LIBS}
)
