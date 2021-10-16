add_library(dubins SHARED "")
target_sources(dubins PRIVATE
    dubins.cpp
    dubins_table_helper.cpp
)
target_include_directories(dubins PUBLIC
    ./ PRIVATE ../
)




