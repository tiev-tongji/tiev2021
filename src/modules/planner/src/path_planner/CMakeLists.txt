add_library(path_planner SHARED "")

target_sources(path_planner PRIVATE
    path_planner.cpp
)

target_include_directories(path_planner PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/path_planner
)

target_link_libraries(path_planner PUBLIC
    planner_common
)
