#pragma once

#include "../../include/planner_common/pose.h"
#include "path_time_graph/path_time_graph.h"
#include "path_time_heuristic/gridded_path_time_graph.h"
#include "qp_spline_speed_optimizer/qp_speed_optimizer.h"
#include "splines/Splines.h"

namespace TiEV {

class SpeedPath {
public:
    bool                        success    = false;
    bool                        qp_success = false;
    vector<STBoundary>          st_boundaries;
    Spline1d                    splines;
    vector<SplineLib::cSpline2> cubic_splines;
    vector<Pose>                path;
    SpeedData                   dp_speed_data;
};

class SpeedOptimizer {
private:
    PathTimeGraph         st_data_;
    GriddedPathTimeGraph  gridded_path_time_graph_;
    std::vector<Obstacle> obstacle_list_;
    std::vector<Pose>&    trajectory_;
    SpeedData             dp_speed_data_;
    SpeedData             spline_speed_data_;

    std::pair<double, double> path_range_;
    std::pair<double, double> time_range_;

    // qp speed optimizer
    QpSpeedOptimizer qp_speed_optimizer_;

public:
    /**
     * @brief Interface of speed optimizer
     * @param obstacle_list
     * @param trajectory
     * @param speed_limit
     * @param total_path_length
     * @return Whether success
     */
    static SpeedPath RunSpeedOptimizer(const std::vector<DynamicObj>& obstacle_list, std::vector<Pose>& trajectory, const std::vector<std::pair<double, double>>& speed_limit,
                                       double total_path_length);

    SpeedOptimizer() = delete;

    // Manually add st_boundaries for test use
    void SetStBoundaries(std::vector<STBoundary>& st_boundaries) {
        st_data_.SetStBoundaries(st_boundaries);
    }

    // Getter
    const SpeedData& dp_speed_data() {
        return dp_speed_data_;
    }

    // Getter
    const PathTimeGraph& st_data() {
        return st_data_;
    }

    // QP result getter
    const Spline1d& splines() {
        return qp_speed_optimizer_.splines();
    }

    /**
     * @brief Constructor and Initialization. Obstacles within
     *        the range of current trajectory will be set on
     *        S-T graph as class STBoundary.
     * @param obstacle_list Obstacles that' ve been perceived.
     * @param trajectory The current trajectory.
     * @param s_start
     * @param s_end
     * @param t_start
     * @param t_end
     */
    SpeedOptimizer(std::vector<Obstacle>& obstacle_list, std::vector<Pose>& trajectory, const std::vector<std::pair<double, double>>& speed_limit, double s_start, double s_end, double t_start,
                   double t_end);

    /**
     * @brief Run DP speed optimizer.
     *        The favorable speed
     *        profile is at member
     *        speed_data_.
     * @return Whether succeed.
     */
    bool DP_Process();

    /**
    * @brief Run QP speed optimizer
    */
    bool QP_Process();

    /**
     * @brief Process altogether: DP + spline
     * @return Whether succeed
     */
    bool Process(SpeedPath& speed_path);

    // Print debug log
    void PrintLog();
};
}
