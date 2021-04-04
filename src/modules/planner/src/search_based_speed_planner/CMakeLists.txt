add_library(search_based_speed_planner SHARED "")
target_sources(search_based_speed_planner PRIVATE
   speed_planner.cpp 
   speed_planner.h
)
