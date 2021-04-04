add_library(reeds_shepp ${reeds_shepp_src})

add_library(reeds_shepp SHARED "")
target_sources(reeds_shepp PRIVATE
    reeds_shepp_table_helper.cpp
    reeds_shepp.cpp
)
target_include_directories(reeds_shepp PRIVATE
    ${CMAKE_SRC_DIR}/include/planner_common
)
