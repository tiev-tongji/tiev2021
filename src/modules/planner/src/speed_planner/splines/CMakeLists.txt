add_library(splines SHARED "")
target_sources(splines PRIVATE
    Splines.cpp
)
target_include_directories(speed PRIVATE
    ${CMAKE_SRC_DIR}/include/speed_planner
)