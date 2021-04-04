add_library(math SHARED "")
target_sources(math PRIVATE
    linear_interpolation.cpp
    path_matcher.cpp
    box.cpp
)
target_include_directories(math PRIVATE
    ${CMAKE_SOURCE_DIR}/include/speed_planner
)
