#pragma once

#include <string>
#include <thread>

#include <eigen3/Eigen/Core>
#include "macros.h"

namespace TiEV {

class SplineSegKernel {
 public:
  // generating kernel matrix
  Eigen::MatrixXd Kernel(const uint32_t num_params, const double accumulated_x);

  // only support N <= 3 cases
  Eigen::MatrixXd NthDerivativeKernel(const uint32_t n,
                                      const uint32_t num_params,
                                      const double accumulated_x);

  std::mutex spline_seg_kernel_mtx;
 private:
  Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                   const double accumulated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                              const double accumulated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                             const double accumulated_x);

  void IntegratedTermMatrix(const uint32_t num_of_params, const double x,
                            const std::string& type,
                            Eigen::MatrixXd* term_matrix) const;
  void CalculateFx(const uint32_t num_of_params);
  void CalculateDerivative(const uint32_t num_of_params);
  void CalculateSecondOrderDerivative(const uint32_t num_of_params);
  void CalculateThirdOrderDerivative(const uint32_t num_of_params);

  const uint32_t reserved_order_ = 5;
  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;

  DECLARE_SINGLETON(SplineSegKernel)
};
} 

