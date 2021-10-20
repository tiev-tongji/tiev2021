add_library(speed SHARED "")
target_sources(speed PRIVATE
    speed_data.cpp
    speed_limit.cpp
)
target_include_directories(speed PUBLIC 
    ./
)
target_link_libraries(speed PUBLIC
    planner_common
    math
)

