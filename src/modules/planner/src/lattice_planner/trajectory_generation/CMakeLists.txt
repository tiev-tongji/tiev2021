add_library(trajectory_generation STATIC "")

target_sources(trajectory_generation PRIVATE
    end_condition_sampler.cpp
    lattice_trajectory1d.cpp
    piecewise_acceleration_trajectory1d.cpp
    piecewise_braking_trajectory_generator.cpp
    trajectory_combiner.cpp
    trajectory1d_generator.cpp
)

target_include_directories(trajectory_generation PUBLIC
    ./
    ../
)

target_link_libraries(trajectory_generation PUBLIC 
    planner_common
    reference_line_info
)