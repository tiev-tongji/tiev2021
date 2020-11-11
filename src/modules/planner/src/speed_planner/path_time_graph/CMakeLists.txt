include_directories(../common)

set(path_time_graph_src ${path_time_graph_src} line_segment.cpp)
set(path_time_graph_src ${path_time_graph_src} path_time_graph.cpp)
set(path_time_graph_src ${path_time_graph_src} st_boundary.cpp)
set(path_time_graph_src ${path_time_graph_src} st_point.cpp)

add_library(path_time_graph ${path_time_graph_src})
