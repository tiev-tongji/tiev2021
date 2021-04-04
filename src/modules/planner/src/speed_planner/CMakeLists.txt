find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
add_subdirectory(path_time_heuristic)
add_subdirectory(speed)
add_subdirectory(path_time_graph)
add_subdirectory(math)
add_subdirectory(qp_spline_speed_optimizer)
add_subdirectory(splines)

add_library(speed_planner SHARED "")
target_sources(speed_planner PRIVATE
    speed_optimizer.cpp
)
target_include_directories(speed_planner PRIVATE
    ${CMAKE_SOURCE_DIR}/include/speed_planner
)
target_link_libraries(speed_planner PRIVATE 
    path_time_heuristic 
    qp_spline_speed_optimizer 
    path_time_graph 
    speed 
    math 
    splines
)
