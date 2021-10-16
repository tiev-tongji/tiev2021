#pragma once

#include <vector>

#include "osqp/osqp.h"
#include "spline_1d_solver.h"

namespace TiEV {

using Eigen::MatrixXd;
template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N> &dense_matrix,
                      std::vector<T> *data, std::vector<D> *indices,
                      std::vector<D> *indptr);

class OsqpSplineSolver:public Spline1dSolver {
public:
    OsqpSplineSolver(const std::vector<double>& x_knots, const uint32_t order);
    virtual ~OsqpSplineSolver();

    bool Solve() override;

    void CleanUp();

    void ResetOsqp();

private:
    OSQPSettings* settings_ = nullptr;
    OSQPWorkspace* work_ = nullptr;
    OSQPData* data_ = nullptr;
};

}

