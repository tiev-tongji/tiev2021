set(path_planner_src "path_planner.cpp")
add_library(path_planner hybrid_astar_planner.cpp local_planning_map.cpp primitive_sets.cpp primitives.cpp path_planner.cpp)

add_executable(path_planner_test "path_planner_test.cpp")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(path_planner_test ${OpenCV_LIBS} path_planner speed_planner message_manager planner_common coordinate_converter zcm)