add_executable(planner main.cpp)
target_link_libraries(planner planner_tools visualization message_manager road_manager path_planner speed_planner decision)

# For speed optimizer test
# It would be nice if you could comment them instead of erasing them
add_executable(speed_test speed_test.cpp)
target_link_libraries(speed_test planner_tools visualization message_manager road_manager path_planner speed_planner decision)

# For osqp solver test
#add_executable(osqp_solver_test osqp_solver_test.cpp)
#target_link_libraries(osqp_solver_test speed_planner visualization)
