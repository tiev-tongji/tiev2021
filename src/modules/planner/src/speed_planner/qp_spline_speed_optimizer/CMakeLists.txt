add_library(qp_spline_speed_optimizer SHARED "")
target_sources(qp_spline_speed_optimizer PRIVATE
    osqp_spline_solver.cpp
    qp_speed_optimizer.cpp
    spline_1d_kernel.cpp
    polynomial_xd.cpp
    spline_seg_kernel.cpp
    spline_1d.cpp
    spline_1d_seg.cpp
    affine_constraint.cpp
    spline_1d_constraint.cpp
)
target_include_directories(qp_spline_speed_optimizer PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/speed_planner
    ${osqp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(qp_spline_speed_optimizer PRIVATE 
    osqp::osqp 
    Eigen3::Eigen
)






