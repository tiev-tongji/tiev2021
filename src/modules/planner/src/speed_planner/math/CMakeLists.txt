add_library(math SHARED "")
target_sources(math PRIVATE
    linear_interpolation.cpp
    path_matcher.cpp
    box.cpp
)
target_include_directories(math PUBLIC 
    ./
)
target_link_libraries(math PUBLIC
    planner_common
)
