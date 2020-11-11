add_subdirectory(reeds_shepp_table)
add_subdirectory(dubins_table)

add_library(distance_table "distance_table.cpp")
target_link_libraries(distance_table reeds_shepp dubins)

add_executable(generate_tables generate_tables.cpp)
target_link_libraries(generate_tables reeds_shepp dubins)
