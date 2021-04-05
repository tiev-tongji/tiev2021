add_subdirectory(look_up_tables)

add_library(planner_common SHARED "")

target_sources(planner_common PRIVATE
    collision_check.cpp
    config.cpp
    tiev_utils.cpp
)

target_include_directories(planner_common PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/planner_common
    ${CMAKE_SOURCE_DIR}/third_party
)

target_link_libraries(planner_common PUBLIC
    distance_table
)