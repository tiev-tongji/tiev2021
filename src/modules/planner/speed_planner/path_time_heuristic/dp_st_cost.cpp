#include "dp_st_cost.h"
#include <cmath>
#include <iostream>

namespace TiEV {

double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
    const double s = st_graph_point.st_point().s();
    const double t = st_graph_point.st_point().t();

    double cost = 0.0;

    for (const auto& obstacle : obstacles_) {
        const STBoundary& boundary = obstacle.st_boundary();
       if (t < boundary.min_t() || t > boundary.max_t()) {
            continue;
        }

        if (boundary.IsPointInBoundary(st_graph_point.st_point())) {
            return std::numeric_limits<double>::infinity();
        }

        double s_upper = 0.0;
        double s_lower = 0.0;

        double r1 = (t - boundary.bottom_left_point().t()) /
                (boundary.bottom_right_point().t() - boundary.bottom_left_point().t());
        s_lower = boundary.bottom_left_point().s() +
                r1 * (boundary.bottom_right_point().s() - boundary.bottom_left_point().s());
        double r2 = (t - boundary.upper_left_point().t()) /
                (boundary.upper_right_point().t() - boundary.upper_left_point().t());
        s_upper = boundary.upper_left_point().s() +
                r2 * (boundary.upper_right_point().s() - boundary.upper_left_point().s());

        if (s < s_lower) {
            const double follow_distance = dp_st_config_.safe_distance() + obstacle.path.front().v * dp_st_config_.safe_time_buffer();
            // const double follow_distance =  obstacle.path.front().v * dp_st_config_.safe_time_buffer();
            if (s + follow_distance < s_lower) {
                continue;
            } else {
                auto s_diff = follow_distance - s_lower + s;
                cost += dp_st_config_.obstacle_weight() * s_diff * s_diff;
            }
        } else if (s > s_upper) {
            if (s > s_upper + dp_st_config_.safe_distance()) {
                continue;
            } else {
                auto s_diff = dp_st_config_.safe_distance() + s_upper - s;
                cost += dp_st_config_.obstacle_weight() * s_diff * s_diff;
            }
        }
    }
    return cost * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint &first, const STPoint &second, const double speed_limit) {
    double cost = 0.0;
    const double speed = (second.s() - first.s()) / unit_t_;

    // Backing up is not allowed(for now)
    if (speed < 0) {
        return std::numeric_limits<double>::infinity();
    }

    double det_speed = (speed - speed_limit) / (speed_limit + 1e-6);
    if (det_speed > 0) {
        cost += dp_st_config_.exceed_speed_penalty() * (det_speed * det_speed) * unit_t_;
    } else if (det_speed < 0) {
        cost += dp_st_config_.low_speed_penalty() * (det_speed * det_speed) * unit_t_;
    }

    return cost;
}

double DpStCost::GetAccelCost(const double accel) {
    double cost = 0.0;
    const double accel_sq = accel * accel;
    double max_acc = dp_st_config_.max_acceleration();
    double max_dec = dp_st_config_.max_deceleration();
    double accel_penalty = dp_st_config_.accel_penalty();
    double decel_penalty = dp_st_config_.decel_penalty();

    if (accel > 0) {
        cost = accel_penalty * accel_sq;
    } else {
        cost = decel_penalty * accel_sq;
    }

    // Baidu Apollo approch
    cost += accel_sq * decel_penalty * decel_penalty /
            (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
            (1 + std::exp(-1.0 * (accel - max_acc)));

    return cost * unit_t_;
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed, const STPoint &first,
                                         const STPoint &second) {
    double current_speed = (second.s() - first.s()) / unit_t_;
    double accel = (current_speed - pre_speed) / unit_t_;
    return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByThreePoints(const STPoint &first, const STPoint &second,
                                           const STPoint &third) {
    double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
    return GetAccelCost(accel);
}

double DpStCost::GetJerkCost(const double jerk) {
    double cost = 0.0;
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
        cost = dp_st_config_.positive_jerk_weight() * jerk_sq;
    } else {
        cost = dp_st_config_.negative_jerk_weight() * jerk_sq;
    }

    return cost * unit_t_;
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc, const STPoint &pre_point,
                                        const STPoint &curr_point) {
    const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
    const double curr_accel = (curr_speed - pre_speed) / unit_t_;
    const double jerk = (curr_accel - pre_acc) / unit_t_;
    return GetJerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed, const STPoint &first_point,
                                          const STPoint &second_point, const STPoint &third_point) {
    const double pre_speed = (second_point.s() - first_point.s()) / unit_t_;
    const double pre_acc = (pre_speed - first_speed) / unit_t_;
    const double curr_speed = (third_point.s() - second_point.s()) / unit_t_;
    const double curr_acc = (curr_speed - pre_speed) / unit_t_;
    const double jerk = (curr_acc - pre_acc) / unit_t_;
    return GetJerkCost(jerk);
}

double DpStCost::GetJerkCostByFourPoints(const STPoint &first, const STPoint &second,
                                         const STPoint &third, const STPoint &fourth) {
    double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                  (unit_t_ * unit_t_ * unit_t_);
    return GetJerkCost(jerk);
}
}
