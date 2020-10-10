#pragma once

#include <string>
#include <vector>
#include "../../message_manager/message_manager.h"
#include "../common/obstacle.h"
#include "../math/box.h"
#include "../math/sl_boundary.h"
#include "st_point.h"
#include "st_boundary.h"


namespace TiEV {

/**
 * @class PathTimeGraph
 * @brief Path-Time graph(S-T graph) for speed planning
 */
class PathTimeGraph {
private:
    std::pair<double , double> time_range_;
    std::pair<double, double> path_range_;
    std::vector<STBoundary> st_boundaries_;

    double path_length_;
    double total_time_;

    const double Default_Path_Width_ = 2;
    const double Trajectory_Time_Resolution = 0.05; // Temporarily set

public:
    PathTimeGraph() {
        time_range_ = std::make_pair(0.0, 0.0);
        path_range_ = std::make_pair(0.0, 0.0);
        path_length_ = 0;
        total_time_ = 0;
    }

    // Manually add st_boundaries for test use
    void SetStBoundaries(const std::vector<STBoundary>& st_boundaries) {
        st_boundaries_ = st_boundaries;
    }

    double path_length() const { return path_length_; }

    double total_time() const { return total_time_; }

    std::vector<STBoundary>& st_boundaries() { return st_boundaries_; }

    PathTimeGraph(std::vector<Obstacle>& obstacles,
            const std::vector<Point>& path,
            double s_start, double s_end,
            double t_start, double t_end);

    /**
     * @brief Get the location(PathPoint) of obstacle at time t
     * @param obstacle
     * @param t Time
     * @return Location(PathPoint) of obstacle at t
     */
    Point GetPointAtTime(const Obstacle& obstacle, double t) const;

    /**
     * @brief Get a bounding box of a static obstacle
     * @param obstacle Static obstacle
     * @return A bounding box
     */
    Box GetStaticBoundingBox(const Obstacle& obstacle);

    /**
     * @brief Get a bounding box based on the given path point
     * @param obstacle Dynamic obstacle
     * @param point Path point
     * @return A bounding box
     */
    Box GetDynamicBoundingBox(const Obstacle& obstacle, const Point& point);

    /**
     * @brief calculate ST boundary of obstacle with given vertices
     * @param vertices
     * @param path
     * @return SL Boundary of obstacle
     */
    SLBoundary ComputeObstacleSLBoundary(const std::vector<Vec>& vertices,
            const std::vector<Point>& path);

private:
    /**
     * @brief Set up obstacles on the s-t graph and get a series of st_boundaries.
     *        Both static obstacles and dynamic obstacles are included.
     * @param obstacles Obstalces list
     * @param path The given path
     */
    void SetupObstacles(std::vector<Obstacle>& obstacles,
            const std::vector<Point>& path);

    void SetDynamicObstacle(Obstacle& obstacle,
            const std::vector<Point>& path);

    void SetStaticObstacle(Obstacle& obstacle, const std::vector<Point>& path);
};
} // namespace TiEV
