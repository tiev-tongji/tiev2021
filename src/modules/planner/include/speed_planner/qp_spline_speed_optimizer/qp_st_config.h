#pragma once

namespace TiEV {

class QpStConfig {
private:
    double   total_time_;
    double   total_path_length_;
    uint32_t number_of_discrete_graph_t_;
    uint32_t spline_order_;
    double   vehicle_upper_speed_limit_;
    double   max_acceleration_;
    double   max_deceleration_;

    // Weight in QP objective function
    double velocity_kernel_weight_;
    double accel_kernel_weight_;
    double jerk_kernel_weight_;
    double dp_st_reference_weight_;

public:
    QpStConfig() {
        total_time_                 = 5;    // s
        total_path_length_          = 100;  // m
        number_of_discrete_graph_t_ = 11;
        spline_order_               = 5;
        vehicle_upper_speed_limit_  = 20;  // m/s
        max_acceleration_           = 20;    // m/s^2
        max_deceleration_           = -1.0;   // m/s^2

        velocity_kernel_weight_ = 0;
        accel_kernel_weight_    = 1000.0;
        jerk_kernel_weight_     = 1000.0;
        dp_st_reference_weight_ = 1000.0;
    }

    const double total_time() const {
        return total_time_;
    }

    const double total_path_length() const {
        return total_path_length_;
    }

    const uint32_t number_of_discrete_graph_t() const {
        return number_of_discrete_graph_t_;
    }

    const uint32_t spline_order() const {
        return spline_order_;
    }

    const double vehicle_upper_speed_limit() const {
        return vehicle_upper_speed_limit_;
    }

    const double max_acceleration() const {
        return max_acceleration_;
    }

    const double max_deceleration() const {
        return max_deceleration_;
    }

    const double velocity_kernel_weight() const {
        return velocity_kernel_weight_;
    }

    const double accel_kernel_weight() const {
        return accel_kernel_weight_;
    }

    const double jerk_kernel_weight() const {
        return jerk_kernel_weight_;
    }

    const double dp_st_reference_weight() const {
        return dp_st_reference_weight_;
    }
};
}
