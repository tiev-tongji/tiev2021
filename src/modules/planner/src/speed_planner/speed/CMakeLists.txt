add_library(speed SHARED "")
target_sources(speed PRIVATE
    speed_data.cpp
    speed_limit.cpp
)
target_include_directories(speed PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/speed_planner
)

