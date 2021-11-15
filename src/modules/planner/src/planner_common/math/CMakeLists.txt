add_subdirectory(curve1d)

add_library(math SHARED "")

target_sources(math PRIVATE
    linear_interpolation.cpp
    path_matcher.cpp
    box.cpp
    cartesian_frenet_conversion.cpp
    vec.cpp
    path_smoother.cpp
)

target_include_directories(math PUBLIC 
    ./
    ${CMAKE_SOURCE_DIR}/src/planner_common
)

target_link_libraries(math PUBLIC
    curve1d
)