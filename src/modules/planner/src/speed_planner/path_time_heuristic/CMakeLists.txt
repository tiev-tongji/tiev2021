add_library(path_time_heuristic SHARED "")
target_sources(path_time_heuristic PRIVATE
    dp_st_cost.cpp
    gridded_path_time_graph.cpp
    st_graph_point.cpp
)
target_include_directories(path_time_heuristic PUBLIC 
    ./
)
target_link_libraries(path_time_heuristic PUBLIC
    path_time_graph
    planner_common
    speed
)