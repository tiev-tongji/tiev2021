add_library(curve1d SHARED "")

target_sources(curve1d PRIVATE
    quartic_polynomial_curve1d.cpp
    quintic_polynomial_curve1d.cpp
)

target_include_directories(curve1d PUBLIC 
    ./
)
