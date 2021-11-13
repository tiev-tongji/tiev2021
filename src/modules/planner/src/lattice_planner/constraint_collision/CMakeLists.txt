add_library(constraint_collision SHARED "")

target_sources(constraint_collision PRIVATE
    constraint_checker.cpp
    constraint_checker1d.cpp
)

target_include_directories(constraint_collision PUBLIC
    ./
)

target_link_libraries(constraint_collision PUBLIC 
    planner_common
)