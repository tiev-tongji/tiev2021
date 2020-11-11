add_subdirectory(look_up_tables)
aux_source_directory(. SRC_DIR)
add_library(planner_common ${SRC_DIR})
target_link_libraries(planner_common distance_table)