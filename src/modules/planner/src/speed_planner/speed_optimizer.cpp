#include "speed_optimizer.h"
#include <iostream>
#include <fstream>

namespace TiEV {

SpeedPath SpeedOptimizer::RunSpeedOptimizer(const std::vector<DynamicObj> &obstacle_list, \
std::vector<Pose> &trajectory, const std::vector<std::pair<double, double> > &speed_limit, \
double total_path_length) {
    // Debug
    // std::cout << "Total length of path: " << trajectory.back().s << std::endl;

	SpeedPath speed_path;
    if (trajectory.empty()) {
        speed_path.success = false;
        return speed_path;
    }
    vector<Obstacle> obj_list;
    for (const auto& obstacle : obstacle_list) {
        Obstacle obj(obstacle);

        // Specially take care of pedestrians for safety's sake
        if (obj.type == 2) { // pedestrian
            // TODO: Need testing in practice
            obj.width = 1;
            obj.length = 1;
        }
        obj_list.emplace_back(obj);
    }
    const double TOTAL_TIME = 5.0;
    SpeedOptimizer speed_optimizer(obj_list, trajectory, speed_limit, 0, total_path_length, 0, TOTAL_TIME);
    if (speed_optimizer.Process(speed_path)) {
        std::cout << "Speed Optimizer succeeds!" << std::endl;
        speed_path.success = true;
    } else {
        std::cout << "Speed Optimizer fails!" << std::endl;
        speed_path.success = false;
    }
	speed_path.st_boundaries = speed_optimizer.st_data_.st_boundaries();
	speed_path.path = trajectory;
	speed_path.dp_speed_data = speed_optimizer.dp_speed_data_;

	return speed_path;
}

SpeedOptimizer::SpeedOptimizer(std::vector<Obstacle>& obstacle_list,
                               std::vector<Pose>& trajectory,
                               const std::vector<std::pair<double, double> >& speed_limit,
                               double s_start, double s_end,
                               double t_start, double t_end) :
                               st_data_(obstacle_list, trajectory, s_start, s_end, t_start, t_end),
                               gridded_path_time_graph_(st_data_, obstacle_list, trajectory.front(), SpeedLimit(speed_limit)),
                               obstacle_list_(obstacle_list),
                               trajectory_(trajectory),
                               path_range_(s_start, s_end),
                               time_range_(t_start, t_end),
							   qp_speed_optimizer_(trajectory.front(), trajectory.back(), SpeedLimit(speed_limit)) {}

bool SpeedOptimizer::DP_Process() {
    return gridded_path_time_graph_.Search(&dp_speed_data_);
}

bool SpeedOptimizer::QP_Process() {
	return qp_speed_optimizer_.Process(st_data_, dp_speed_data_);
}

bool SpeedOptimizer::Process(SpeedPath &speed_path) {
    if (!DP_Process()) {
        std::cout << "DP speed optimizer fails!" << std::endl;

        return false;
    } else {
        std::cout << "DP speed optimizer succeeds!" << std::endl;
    }
    if (trajectory_.back().s > 5.0 && QP_Process() && qp_speed_optimizer_.RetrieveSpeedData(spline_speed_data_)) {
        std::cout << "QP speed optimizer succeeds!" << std::endl;
		speed_path.qp_success = true;
		speed_path.splines = splines();
    } else {
        // If failing to solve qp problem,
        // we adopt an alternative method
        // to generate cubic splines
        std::cout << "QP speed optimizer fails, trying alternative method" << std::endl;
        const int NUM_POINT = dp_speed_data_.size();
        SplineLib::Vec2f points[NUM_POINT];
        SplineLib::Vec2f velocity[NUM_POINT];
        SplineLib::cSpline2 splines[NUM_POINT - 1];

        for (size_t i = 0; i < dp_speed_data_.size(); ++i) {
            points[i] = SplineLib::Vec2f(dp_speed_data_[i].t(),
                                         dp_speed_data_[i].s());
            velocity[i] = SplineLib::Vec2f(dp_speed_data_[i].t(),
                                           dp_speed_data_[i].v());
        }
        int numSplines = SplineLib::SplinesFromHermite(NUM_POINT, points, velocity, splines);
        speed_path.cubic_splines.clear();
		for(int i = 0; i < numSplines; i++){
			speed_path.cubic_splines.push_back(splines[i]);
		}
        const float T_RESOLUTION = 0.005;
        for (size_t i = 0; i < NUM_POINT - 1; ++i) {
            float relative_time = 0.0;
            while (relative_time < 1.0) {
                double s = SplineLib::Position(splines[i], relative_time).y;
                double v = SplineLib::Velocity(splines[i], relative_time).y;
                double a = SplineLib::Acceleration(splines[i], relative_time).y;
                double t = relative_time;
                spline_speed_data_.AppendSpeedPoint(s, t, v, a, 0);
                relative_time += T_RESOLUTION;
            }
        }
    }
	
    for (auto& point : trajectory_) {
        point.v = spline_speed_data_.GetVelocityByS(point.s);
        point.a = spline_speed_data_.GetAccelByS(point.s);
        point.t = spline_speed_data_.GetTimeByS(point.s);
    }

    // Debug
    /*
    std::cout << "finally retrieved result: " << std::endl;
    for (const auto& point : trajectory_) {
        std::cout << "s = " << point.s << ' ' <<  "v = " << point.v << std::endl;
    }
    */
    return true;
}


}

