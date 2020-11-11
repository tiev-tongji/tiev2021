find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
add_subdirectory(path_time_heuristic)
add_subdirectory(speed)
add_subdirectory(path_time_graph)
add_subdirectory(math)
add_subdirectory(qp_spline_speed_optimizer)
add_subdirectory(splines)

set(speed_planner_src ${speed_planner_src} speed_optimizer.cpp)

add_library(speed_planner ${speed_planner_src})                                                      

target_link_libraries(speed_planner path_time_heuristic qp_spline_speed_optimizer path_time_graph speed math splines)
