add_subdirectory(constraint_collision)
add_subdirectory(reference_line_info)
add_subdirectory(trajectory_generation)
find_package(OpenCV REQUIRED)

add_library(lattice_planner SHARED "")

target_sources(lattice_planner PRIVATE
    lattice_planner.cpp
)

target_include_directories(lattice_planner PUBLIC 
    ./
)

target_link_libraries(lattice_planner PUBLIC 
    constraint_collision
    reference_line_info
    trajectory_generation
    planner_common
    ${OpenCV_LIBS}
)