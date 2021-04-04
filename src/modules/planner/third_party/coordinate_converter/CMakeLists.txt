add_library(coordinate_converter SHARED "")
    target_sources(coordinate_converter PRIVATE
    basic_coordinate_converter.cpp
    coordinate_converter.cpp
    basic_coordinate_converter.h
    coordinate_converter.h
    angle.cpp
    angle.h
)

