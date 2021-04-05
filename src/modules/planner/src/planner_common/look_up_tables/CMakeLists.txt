add_subdirectory(reeds_shepp_table)
add_subdirectory(dubins_table)

add_library(distance_table SHARED "")
target_sources(distance_table PRIVATE
    distance_table.cpp
)
target_include_directories(distance_table PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/planner_common
)
target_link_libraries(distance_table PUBLIC 
    reeds_shepp
    dubins
)

add_executable(generate_tables
    generate_tables.cpp
)
target_include_directories(generate_tables PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/planner_common
)
target_link_libraries(generate_tables PUBLIC 
    reeds_shepp 
    dubins
)
