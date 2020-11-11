#include "qp_spline_speed_optimizer/osqp_spline_solver.h"
#include <fstream>
#include <iostream>

using namespace std;

namespace TiEV {

using Eigen::MatrixXd;
template <typename T, int M, int N, typename D> void DenseToCSCMatrix(const Eigen::Matrix<T, M, N>& dense_matrix, std::vector<T>* data, std::vector<D>* indices, std::vector<D>* indptr) {
    constexpr double epsilon    = 1e-9;
    int              data_count = 0;
    for(int c = 0; c < dense_matrix.cols(); ++c) {
        indptr->emplace_back(data_count);
        for(int r = 0; r < dense_matrix.rows(); ++r) {
            if(std::fabs(dense_matrix(r, c)) < epsilon) {
                continue;
            }
            data->emplace_back(dense_matrix(r, c));
            ++data_count;
            indices->emplace_back(r);
        }
    }
    indptr->emplace_back(data_count);

// Debug
#if 0
  cout << "Data: ";
  for (size_t i = 0; i < data->size(); ++i) cout << (*data)[i] << ' ';
  cout << "\nindices: ";
  for (size_t i = 0; i < indices->size(); ++i) cout << (*indices)[i] << ' ';
  cout << "\nindptr: ";
  for (size_t i = 0; i < indptr->size(); ++i) cout << (*indptr)[i] << ' ';
  cout << endl;
#endif
}

OsqpSplineSolver::OsqpSplineSolver(const std::vector<double>& x_knots, const uint32_t order) : Spline1dSolver(x_knots, order) {
    // Problem settings
    settings_ = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

    // Define Solver settings as default
    osqp_set_default_settings(settings_);
    settings_->alpha      = 1.0;
    settings_->eps_abs    = 1.0e-03;
    settings_->eps_rel    = 1.0e-03;
    settings_->max_iter   = 5000;
    settings_->verbose    = false;
    settings_->warm_start = true;

    data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
}

OsqpSplineSolver::~OsqpSplineSolver() {
    CleanUp();
}

void OsqpSplineSolver::CleanUp() {
    osqp_cleanup(work_);
    if(data_ != nullptr) {
        c_free(data_->A);
        c_free(data_->P);
        c_free(data_);
    }
    if(settings_ != nullptr) {
        c_free(settings_);
    }
}

void OsqpSplineSolver::ResetOsqp() {
    // Problem settings
    settings_ = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    // Populate data
    data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
}

bool OsqpSplineSolver::Solve() {
    // Change P to csc format
    MatrixXd& P = kernel_.kernel_matrix();
    if(P.rows() == 0) {
        return false;
    }

    // Convert P into upper triangular
    for(size_t r = 0; r < P.rows(); ++r) {
        for(size_t c = 0; c < r; ++c) {
            P(r, c) = 0.0;
        }
    }

    // Debug
    ofstream outfile;
    outfile.open("/home/autolab/P_matrix.txt");
    outfile << P << endl;
    outfile.close();
    //

    std::vector<c_float> P_data;
    std::vector<c_int>   P_indices;
    std::vector<c_int>   P_indptr;
    DenseToCSCMatrix(P, &P_data, &P_indices, &P_indptr);

    // Change A to csc format
    const MatrixXd& inequality_constraint_matrix = constraint_.inequality_constraint().constraint_matrix();
    const MatrixXd& equality_constraint_matrix   = constraint_.equality_constraint().constraint_matrix();
    MatrixXd        A(inequality_constraint_matrix.rows() + equality_constraint_matrix.rows(), inequality_constraint_matrix.cols());
    A << inequality_constraint_matrix, equality_constraint_matrix;
    if(A.rows() == 0) {
        return false;
    }

    std::vector<c_float> A_data;
    std::vector<c_int>   A_indices;
    std::vector<c_int>   A_indptr;
    DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

    // Set q, l, u: l < A < u
    const MatrixXd& q_eigen = kernel_.offset();
    c_float         q[q_eigen.rows()];
    for(int i = 0; i < q_eigen.size(); ++i) {
        q[i] = q_eigen(i);
    }

    const MatrixXd& inequality_constraint_boundary = constraint_.inequality_constraint().constraint_boundary();
    const MatrixXd& equality_constraint_boundary   = constraint_.equality_constraint().constraint_boundary();

    int constraint_num = static_cast<int>(inequality_constraint_boundary.rows() + equality_constraint_boundary.rows());

    constexpr double kEpsilon    = 1e-9;
    constexpr float  kUpperLimit = 1e9;

    c_float l[constraint_num];
    c_float u[constraint_num];

    for(int i = 0; i < constraint_num; ++i) {
        if(i < inequality_constraint_boundary.rows()) {
            l[i] = inequality_constraint_boundary(i, 0);
            u[i] = kUpperLimit;
        }
        else {
            const int idx = i - static_cast<int>(inequality_constraint_boundary.rows());
            l[i]          = equality_constraint_boundary(idx, 0) - kEpsilon;
            u[i]          = equality_constraint_boundary(idx, 0) + kEpsilon;
        }
    }

    data_->n = P.rows();
    data_->m = constraint_num;
    data_->P = csc_matrix(data_->n, data_->n, P_data.size(), P_data.data(), P_indices.data(), P_indptr.data());
    data_->q = q;
    data_->A = csc_matrix(data_->m, data_->n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data());
    data_->l = l;
    data_->u = u;

    // work_ = osqp_setup(data_, settings_);
    c_int exitflag = osqp_setup(&work_, data_, settings_);

    // Solve problem
    osqp_solve(work_);

    MatrixXd solved_params = MatrixXd::Zero(P.rows(), 1);
    for(int i = 0; i < P.rows(); ++i) {
        solved_params(i, 0) = work_->solution->x[i];
    }

    last_num_param_      = static_cast<int>(P.rows());
    last_num_constraint_ = constraint_num;

    return spline_.SetSplineSegs(solved_params, spline_.spline_order());
}
}
