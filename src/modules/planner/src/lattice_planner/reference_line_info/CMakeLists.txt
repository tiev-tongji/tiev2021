add_library(reference_line_info SHARED "")

target_sources(reference_line_info PRIVATE
    reference_line_info.cpp
)

target_include_directories(reference_line_info PUBLIC
    ./
)

target_link_libraries(reference_line_info PUBLIC 
    planner_common
    map_manager
)