add_library(path_time_graph SHARED "")

target_sources(path_time_graph PRIVATE
    line_segment.cpp
    path_time_graph.cpp
    st_boundary.cpp
    st_point.cpp
)

target_include_directories(path_time_graph PUBLIC 
    ./
    ${CMAKE_SOURCE_DIR}/src/lattice_planner
)

target_link_libraries(path_time_graph PUBLIC
    math
)