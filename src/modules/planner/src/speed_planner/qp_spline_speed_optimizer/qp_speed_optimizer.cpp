#include "qp_spline_speed_optimizer/qp_speed_optimizer.h"
#include <iostream>

namespace TiEV {
QpSpeedOptimizer::QpSpeedOptimizer(const Pose& init_point, const Pose& target_point, const SpeedLimit& speed_limit)
    : qp_st_config_(), init_point_(init_point), target_point_(target_point), splines_(),
      t_knots_resolution_(qp_st_config_.total_time() / (static_cast<int>(qp_st_config_.number_of_discrete_graph_t()) - 1)), speed_limit_(speed_limit) {

    Init();
}

void QpSpeedOptimizer::Init() {
    // Init knots
    double   curr_t     = 0.0;
    uint32_t num_spline = qp_st_config_.number_of_discrete_graph_t() - 1;
    for(uint32_t i = 0; i <= num_spline; ++i) {
        t_knots_.push_back(curr_t);
        curr_t += t_knots_resolution_;
    }

    // uint32_t num_evaluated_t = 2 * num_spline + 1;
    uint32_t num_evaluated_t = 4 * num_spline + 1;

    // Init evaluated t positions
    curr_t                  = 0;
    t_evaluated_resolution_ = qp_st_config_.total_time() / (num_evaluated_t - 1);
    for(uint32_t i = 0; i < num_evaluated_t; ++i) {
        t_evaluated_.push_back(curr_t);
        curr_t += t_evaluated_resolution_;
    }
}

bool QpSpeedOptimizer::Process(PathTimeGraph& st_graph_data, const SpeedData& reference_dp_speed_points) {
    std::pair<double, double> accel_bound = std::make_pair(qp_st_config_.max_acceleration(), qp_st_config_.max_deceleration());

    // Debug
    /*
    std::cout << "t_knots: " << std::endl;
    for (size_t i = 0; i < t_knots_.size(); ++i) {
        std::cout << t_knots_[i] << ' ';
    }
    std::cout << std::endl;

    std::cout << "t_evaluated: " << std::endl;
    for (size_t i = 0; i < t_evaluated_.size(); ++i) {
        std::cout << t_evaluated_[i] << ' ';
    }
    std::cout << std::endl;
    */

    // Initial osqp spline solver
    OsqpSplineSolver pg(t_knots_, 5);

    auto* spline_constraint = pg.mutable_spline_constraint();
    auto* spline_kernel     = pg.mutable_spline_kernel();

    /*************ADD CONSTRAINT*************/

    // Debug
    // std::cout << "**********QP CONSTRAINT START**********" << std::endl;

    /**
     * Add S(distance) constraint
     */
    std::vector<double> s_upper_bound;
    std::vector<double> s_lower_bound;
    for(const double curr_t : t_evaluated_) {
        double lower_s = 0.0;
        double upper_s = 0.0;
        GetSConstraintByTime(st_graph_data.st_boundaries(), curr_t, qp_st_config_.total_path_length(), &upper_s, &lower_s);
        s_upper_bound.push_back(upper_s);
        s_lower_bound.push_back(lower_s);
    }
    if(!spline_constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound)) {
        std::cout << "QpSpeedOptimizer: ";
        std::cout << "Fail to apply distance constraints!" << std::endl;
        return false;
    }

    /**
     * Add speed constraint
     */
    std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);
    std::vector<double> speed_upper_bound;

    std::cout << "DEBUG: speed upper bound=";
    for(auto const& t : t_evaluated_) {
        double path_s  = reference_dp_speed_points.GetSByTime(t);
        double upper_v = speed_limit_.GetSpeedLimit(path_s);
        speed_upper_bound.emplace_back(upper_v);
        std::cout << upper << " ";
    }
    std::cout << std::endl;

    if(!spline_constraint->AddDerivativeBoundary(t_evaluated_, speed_lower_bound, speed_upper_bound)) {
        std::cout << "QpSpeedOptimizer: Fail to apply speed constraint!" << std::endl;
        return false;
    }

    /**
    * Add acceleration constraint
    */
    std::vector<double> accel_lower_bound(t_evaluated_.size(), accel_bound.second);
    std::vector<double> accel_upper_bound(t_evaluated_.size(), accel_bound.first);
#if 0
    std::cout << "QP acceleration constarint: " << std::endl;
    for(size_t i = 0; i < t_evaluated_.size(); ++i) {
        std::cout << "time = " << t_evaluated_[i] << " accel_bound = " << accel_lower_bound[i] << " " << accel_upper_bound[i] << std::endl;
    }
    std::cout << std::endl;
#endif
    if(!spline_constraint->AddSecondDerivativeBoundary(t_evaluated_, accel_lower_bound, accel_upper_bound)) {
        std::cout << "QpSpeedOptimizer: Fail to apply acceleration constraint!" << std::endl;
        return false;
    }

    /**
     * Add smooth constraint
     */
    if(!spline_constraint->AddSmoothConstraint()) {
        std::cout << "QpSpeedOptimizer: Fail to apply Smooth constraint!" << std::endl;
        return false;
    }
    if(!spline_constraint->AddDerivativeSmoothConstraint()) {
        std::cout << "QpSpeedOptimizer: Fail to apply derivative smooth constraint!" << std::endl;
        return false;
    }
    if(!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
        std::cout << "QpSpeedOptimizer: Fail to apply second derivative smooth constraint!" << std::endl;
        return false;
    }
    if(!spline_constraint->AddThirdDerivativeSmoothConstraint()) {
    std:
        cout << "QpSpeedOptimizer: Fail to apply third derivative smooth constraint!" << std::endl;
        return false;
    }

    /**
     * Add monotone inequality constraint
     */
    if(!spline_constraint->AddMonotoneInequalityConstraintAtKnots()) {
        std::cout << "QpSpeedOptmizer: Fail to apply monotone inequality constarint!" << std::endl;
        return false;
    }

    /**
     * Add initial point constraint: start at 0 m and 0 s
     */
    if(!spline_constraint->AddPointConstraint(0.0, 0.0)) {
        std::cout << "QpSpeedOptimizer: Fail to apply first point constraint!" << std::endl;
        return false;
    }

    /**
     * Add initial point speed constraint
     */
    if(!spline_constraint->AddPointDerivativeConstraint(0.0, init_point_.v)) {
        std::cout << "DEBUG: Initial speed=" << init_point_.v << std::endl;
        std::cout << "QpSpeedOptimizer: Fail to apply initial speed constraint!" << std::endl;
        return false;
    }
#if 1
    if(target_point_.s - reference_dp_speed_points.back().s() <= 1) {
        double last_s = reference_dp_speed_points.back().s();
        double last_v = sqrt(max(0.0, init_point_.v * init_point_.v + 2 * qp_st_config_.max_deceleration() * last_s));
        if(!spline_constraint->AddPointDerivativeConstraint(reference_dp_speed_points.back().t(), last_v)) {
            std::cout << "QpSpeedOptimizer: Fail to apply target speed constraint!" << std::endl;
            return false;
        }
    }
#endif

    // Debug
    // std::cout << "QP The speed of the first point: " << init_point_.v << std::endl << std::endl;

    // spline_constraint->AddPointSecondDerivativeConstraint(0.0, init_point_.a);

    /*************ADD GOAL FUNCTION*************/
    spline_kernel->AddSecondOrderDerivativeMatrix(1000.0);
    spline_kernel->AddThirdOrderDerivativeMatrix(1000.0);

    std::vector<double> fx_guide;
    /*
    for (auto speedpoint : reference_dp_speed_points) {
        fx_guide.emplace_back(speedpoint.s());
    }
    vector<double> x_guide;
    for (size_t i = 0; i < fx_guide.size(); ++i) {
        x_guide.push_back(t_knots_[i]);
    }
     */
    for(size_t i = 0; i < t_knots_.size(); ++i) {
        if(i >= reference_dp_speed_points.size()) {
            fx_guide.emplace_back(reference_dp_speed_points.back().s());
        }
        else {
            fx_guide.emplace_back(reference_dp_speed_points[i].s());
        }
    }

    if(!spline_kernel->AddReferenceLineKernelMatrix(t_knots_, fx_guide, 1000)) {
        std::cout << "Fail to add reference dp speed kernel!" << std::endl;
        return false;
    }
    spline_kernel->AddRegularization(1.0);

    // std::cout << "**********QP CONSTRAINT END**********" << std::endl;

    /*************SOLVE QP PROBLEM*************/
    if(pg.Solve()) {
        std::cout << "QP Solver Good" << std::endl;
        splines_ = pg.spline();
        return true;
    }
    else {
        std::cout << "QP Solver Fail" << std::endl;
        return false;
    }
}

bool QpSpeedOptimizer::RetrieveSpeedData(SpeedData& speed_data) {
    speed_data.clear();
    const double T_RESOLUTION  = 0.005;  // s
    double       relative_time = 0.0;
    while(relative_time <= qp_st_config_.total_time()) {
        double s = splines_(relative_time);
        double t = relative_time;
        double v = splines_.Derivative(relative_time);
        if(v > 100) return false;
        double a  = splines_.SecondOrderDerivative(relative_time);
        double da = splines_.ThirdOrderDerivative(relative_time);
        speed_data.push_back(SpeedPoint(s, t, v, a, da));
        relative_time += T_RESOLUTION;
    }

    // Debug
    /*
    std::cout << "Some samples point from splines result: " << std::endl;
    double t = 0.0;
    for (int i = 0; i < 20; i ++) {
        std::cout << "t = " << t << " s = " << splines_(t) <<  " v = " << splines_.Derivative(t) << std::endl;
        t += 0.25;
    }
    */
    return true;
}

bool QpSpeedOptimizer::GetSConstraintByTime(const std::vector<STBoundary> boundaries, const double time, const double total_path_length, double* const s_upper_bound,
                                            double* const s_lower_bound) const {
    *s_upper_bound = total_path_length;

    for(auto boundary : boundaries) {
        double s_upper = 0.0;
        double s_lower = 0.0;

        if(!boundary.GetUnblockSRange(time, total_path_length, &s_upper, &s_lower)) {
            std::cout << "Get unblock s range failed" << std::endl;
            continue;
        }

        if(boundary.boundary_type() == STBoundary::BoundaryType::YIELD) {
            *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
        }
        else if(boundary.boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
            *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
        }
    }
    return true;
}
}
