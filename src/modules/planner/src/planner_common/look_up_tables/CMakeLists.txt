add_subdirectory(reeds_shepp_table)
add_subdirectory(dubins_table)

add_library(distance_table SHARED "")
target_sources(distance_table PRIVATE
    distance_table.cpp
)
target_include_directories(distance_table PRIVATE
    ${CMAKE_SOURCE_DIR}/include/planner_common
)
target_link_libraries(distance_table PRIVATE
    reeds_shepp
    dubins
)

add_executable(generate_tables PRIVATE 
    generate_tables.cpp
)
target_include_directories(generate_tables PRIVATE
    ${CMAKE_SOURCE_DIR}/include/planner_common
)
target_link_libraries(generate_tables PRIVATE 
    reeds_shepp 
    dubins
)
