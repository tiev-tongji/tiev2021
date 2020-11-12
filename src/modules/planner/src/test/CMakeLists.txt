aux_source_directory(. SRC_DIR)

add_executable(test ${SRC_DIR})
target_link_libraries(test decision path_planner speed_planner map_manager message_manager planner_common coordinate_converter zcm )