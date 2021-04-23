add_library(search_based_speed_planner SHARED "")
target_sources(search_based_speed_planner PRIVATE
   speed_planner.cpp 
   speed_planner.h
)
target_include_directories(search_based_speed_planner PRIVATE
    ${CMAKE_SOURCE_DIR}/include/planner_common
)
