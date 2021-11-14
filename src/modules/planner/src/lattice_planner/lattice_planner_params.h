#pragma once

#include "const.h"

// path_time_graph s and t axis length
constexpr double FLAGS_trajectory_length_horizon  = 500;  // unit grid
constexpr double FLAGS_trajectory_time_horizon    = 10;   // 10s
constexpr double FLAGS_numerical_epsilon          = 1e-6;
constexpr double FLAGS_trajectory_time_resolution = 0.2;

// in end_condition_sampler
// sample parameter for lon trajectory of cruising
constexpr size_t FLAGS_num_of_time_samples     = 2;
constexpr size_t FLAGS_num_velocity_sample     = 2;
constexpr double FLAGS_min_velocity_sample_gap = 10;
// minimum length of x axis required for polynomial generation
constexpr double FLAGS_minimal_time = 5;
// sample parameter for lon trajectory of follow and overtake driving
constexpr double FLAGS_time_min_gap             = 1;  // unit s
constexpr double FLAGS_vehicle_front_to_center_dist = 5;  // unit grid
constexpr double FLAGS_follow_overtake_lon_buffer   = 5;  // unit grid
constexpr double FLAGS_num_follow_samples           = 3;
// sample parameter for lat trajectory
constexpr std::array<double, 1> FLAGS_end_s_candidates = {350};
constexpr std::array<double, 1> FLAGS_end_d_candidates = {0};

// in trajectory_evaluater
constexpr double FLAGS_weight_lon_objective            = 1;
constexpr double FLAGS_weight_lon_collision            = 1;
constexpr double FLAGS_weight_lat_offset               = 1;
constexpr double FLAGS_weight_lon_jerk                 = 1;
constexpr double FLAGS_weight_centripetal_acceleration = 1;
constexpr double FLAGS_weight_lat_comfort              = 1;
// lon_objective cost
constexpr double FLAGS_weight_target_speed   = 1;
constexpr double FLAGS_weight_dist_travelled = 1;
// lon_collision cost
constexpr double FLAGS_lon_collision_yield_buffer    = 5;  // unit grid
constexpr double FLAGS_lon_collision_overtake_buffer = 5;  // unit grid
constexpr double FLAGS_lon_collision_cost_std        = 1;
// lat_offset
constexpr double FLAGS_lat_offset_bound            = 10;  // unit grid
constexpr double FLAGS_trajectory_space_resolution = 5;   // unit grid
constexpr double FLAGS_weight_same_side_offset     = 1;
constexpr double FLAGS_weight_opposite_side_offset = 2;
// lon_jerk cost
constexpr double FLAGS_longitudinal_jerk_upper_bound = 5;
// GuideVelocity
constexpr double FLAGS_lattice_stop_buffer                   = 5;
constexpr double FLAGS_longitudinal_acceleration_upper_bound = 5;
constexpr double FLAGS_comfort_acceleration_factor           = 5;
constexpr double FLAGS_longitudinal_acceleration_lower_bound = 5;
