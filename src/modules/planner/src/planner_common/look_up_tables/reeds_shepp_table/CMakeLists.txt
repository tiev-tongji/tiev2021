add_library(reeds_shepp SHARED "")
target_sources(reeds_shepp PRIVATE
    reeds_shepp_table_helper.cpp
    reeds_shepp.cpp
)
target_include_directories(reeds_shepp PUBLIC
    ./ PRIVATE ../
)
