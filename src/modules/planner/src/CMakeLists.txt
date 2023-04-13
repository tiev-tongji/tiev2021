add_subdirectory(protos)
add_subdirectory(planner_common)
add_subdirectory(message_manager)
add_subdirectory(planner_context)
add_subdirectory(map_manager)
add_subdirectory(speed_planner)
add_subdirectory(path_planner)
add_subdirectory(CloudTracking)
add_subdirectory(decision)
add_subdirectory(test)
add_subdirectory(visualization)
add_subdirectory(routing)
add_subdirectory(search_based_speed_planner)
add_subdirectory(lattice_planner)

add_executable(planner main.cpp)
target_link_libraries(planner PUBLIC 
    decision 
    path_planner 
    search_based_speed_planner 
    speed_planner 
    map_manager 
    planner_context
    message_manager 
    routing 
    planner_common 
    coordinate_converter 
    zcm 
    lattice_planner
)
