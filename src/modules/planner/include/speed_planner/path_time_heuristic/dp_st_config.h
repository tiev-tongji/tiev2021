#pragma once

#include <stdint-gcc.h>

namespace TiEV {

class DpStConfig {
private:
    uint32_t matrix_dimension_s_;

    uint32_t matrix_dimension_t_;

    double vehicle_upper_speed_limit_; // The upper speed limit of the ego vehicle

    double safe_time_buffer_; // For estimating follow distance

    double safe_distance_; // The safe distance when ahead of a obstacle

    double obstacle_weight_; // Weight of obstacle in cost calculation

    double exceed_speed_penalty_; // Penalty for exceeding speed limit

    double low_speed_penalty_; // Penalty for being slower than speed limit

    double max_acceleration_; // The max acceleration of ego vehicle

    double max_deceleration_; // The max deceleration of ego vehicle

    double accel_penalty_; // Penalty for acceleration

    double decel_penalty_; // Penalty for deceleration

    double positive_jerk_weight_; // Weight of positive jerk

    double negative_jerk_weight_; // Weight of negative jerk
public:
    // TODO(Charles Peng): Use JSON configuration file, and parse it.
    DpStConfig() {
        matrix_dimension_s_ = 501; 
        matrix_dimension_t_ = 11;
        vehicle_upper_speed_limit_ = 16.66;
        safe_time_buffer_ = 3.0;
        // safe_distance_ = 3.0;
        safe_distance_ = 3.0;
        obstacle_weight_ = 1.0;
       //  exceed_speed_penalty_ = 0.5;
        exceed_speed_penalty_ = 100;
        low_speed_penalty_ = 100;
        max_acceleration_ = 3.0;
        max_deceleration_ = -1.0;
        accel_penalty_ = 0;
        decel_penalty_ = 2;
        positive_jerk_weight_ = 1.0;
        negative_jerk_weight_ = 1.0;
    }

    DpStConfig(const DpStConfig& dp_st_config) = default;

    uint32_t matrix_dimension_s() const { return matrix_dimension_s_; }

    uint32_t matrix_dimension_t() const { return matrix_dimension_t_; }

    double vehicle_upper_speed_limit() const { return vehicle_upper_speed_limit_; }

    double safe_time_buffer() const { return safe_time_buffer_; }

    double safe_distance() const { return safe_distance_; }

    double obstacle_weight() const { return obstacle_weight_; }

    double exceed_speed_penalty() const { return exceed_speed_penalty_; }

    double low_speed_penalty() const { return low_speed_penalty_; }

    double max_acceleration() const { return max_acceleration_; }

    double max_deceleration() const { return max_deceleration_; }

    double accel_penalty() const { return accel_penalty_; }

    double decel_penalty() const { return decel_penalty_; }

    double positive_jerk_weight() const { return positive_jerk_weight_; }

    double negative_jerk_weight() const { return negative_jerk_weight_; }
};
}
