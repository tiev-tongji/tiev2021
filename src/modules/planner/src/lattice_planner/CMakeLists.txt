add_subdirectory(constraint_collision)
add_subdirectory(trajectory_generation)
add_subdirectory(reference_line)

add_library(lattice_planner SHARED "")
target_sources(lattice_planner PRIVATE
    lattice_planner.cpp
)
target_include_directories(lattice_planner PUBLIC 
    ./
)
target_link_libraries(lattice_planner PUBLIC 
    constraint_collision
    trajectory_generation
    reference_line
    planner_common
)