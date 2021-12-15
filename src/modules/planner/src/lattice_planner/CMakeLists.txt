add_subdirectory(reference_line_info)
add_subdirectory(trajectory_generation)
find_package(OpenCV REQUIRED)

add_library(lattice_planner STATIC "")

target_sources(lattice_planner PRIVATE
    lattice_planner.cpp
)

target_include_directories(lattice_planner PUBLIC 
    ./
)

target_link_libraries(lattice_planner PUBLIC 
    reference_line_info
    trajectory_generation
    planner_common
    map_manager
    ${OpenCV_LIBS}
)