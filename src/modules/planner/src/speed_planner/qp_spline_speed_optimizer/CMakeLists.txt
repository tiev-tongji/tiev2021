include_directories(${osqp_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} osqp_spline_solver.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} qp_speed_optimizer.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} spline_1d_kernel.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} polynomial_xd.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} spline_seg_kernel.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} spline_1d.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} spline_1d_seg.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} affine_constraint.cpp)
set(qp_spline_speed_optimizer ${qp_spline_speed_optimizer} spline_1d_constraint.cpp)

add_library(qp_spline_speed_optimizer ${qp_spline_speed_optimizer})

target_link_libraries(qp_spline_speed_optimizer PRIVATE osqp::osqp Eigen3::Eigen)






