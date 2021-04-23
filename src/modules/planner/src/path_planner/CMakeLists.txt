add_library(path_planner SHARED "")

target_sources(path_planner PRIVATE
    hybrid_astar_planner.cpp
    analytic_expansion_providers.cpp
    local_planning_map.cpp
    primitive_sets.cpp
    primitives.cpp
    path_planner.cpp
)

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

target_include_directories(path_planner PUBLIC
    ${CMAKE_SOURCE_DIR}/include/path_planner
)

target_link_libraries(path_planner
    PRIVATE
        ${G1FITTING_LIBRARY}
    PUBLIC
        speed_planner
        message_manager
        planner_common
        ${STEERING_LIBRARY}
)

find_package(OpenCV REQUIRED)

add_executable(path_planner_test "")

target_sources(path_planner_test PRIVATE
    "path_planner_test.cpp"
)

target_include_directories(path_planner_test PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(path_planner_test PRIVATE
    ${OpenCV_LIBS}
    path_planner
)
