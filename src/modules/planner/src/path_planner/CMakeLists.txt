set(PATH_PLANNER_SRC hybrid_astar_planner.cpp analytic_expansion_providers.cpp local_planning_map.cpp primitive_sets.cpp primitives.cpp path_planner.cpp)
add_library(path_planner ${PATH_PLANNER_SRC})

find_library(G1FITTING_LIBRARY Clothoid)
if(NOT G1FITTING_LIBRARY)
    message(FATAL_ERROR "Library G1fitting not found. Make and install third_party/G1fitting first.")
else()
    message(STATUS "Found G1fitting: " ${G1FITTING_LIBRARY})
endif()

find_library(STEERING_LIBRARY steering_functions)
if(NOT STEERING_LIBRARY)
    message(FATAL_ERROR "Library steering_functions not found. Make and install third_party/steering_functions first.")
else()
    message(STATUS "Found steering_functions: " ${STEERING_LIBRARY})
endif()

target_link_libraries(path_planner ${G1FITTING_LIBRARY} ${STEERING_LIBRARY})

add_executable(path_planner_test "path_planner_test.cpp")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(path_planner_test ${OpenCV_LIBS} path_planner speed_planner message_manager planner_common coordinate_converter zcm)