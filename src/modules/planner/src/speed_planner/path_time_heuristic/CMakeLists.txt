add_library(path_time_heuristic SHARED "")
target_sources(path_time_heuristic PRIVATE
    dp_st_cost.cpp
    gridded_path_time_graph.cpp
    st_graph_point.cpp
)
target_include_directories(path_time_heuristic PRIVATE
    ${CMAKE_SOURCE_DIR}/include/speed_planner
)