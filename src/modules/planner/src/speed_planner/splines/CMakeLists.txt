add_library(splines SHARED "")
target_sources(splines PRIVATE
    Splines.cpp
)
target_include_directories(splines PUBLIC 
    ./
)