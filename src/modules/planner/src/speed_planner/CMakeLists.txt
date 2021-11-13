find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
add_subdirectory(path_time_heuristic)
add_subdirectory(speed)
add_subdirectory(qp_spline_speed_optimizer)
add_subdirectory(splines)

add_library(speed_planner SHARED "")
target_sources(speed_planner PRIVATE
    speed_optimizer.cpp
)
target_include_directories(speed_planner PUBLIC 
    ./
)
target_link_libraries(speed_planner PUBLIC 
    path_time_heuristic 
    qp_spline_speed_optimizer 
    speed 
    splines
    planner_common
)
