#pragma once

#include <string>
#include <vector>

#include "box.h"
#include "collision_check.h"
#include "obstacle.h"
#include "sl_boundary.h"
#include "st_boundary.h"
#include "st_point.h"
#include "tiev_class.h"

namespace TiEV {

/**
 * @class PathTimeGraph
 * @brief Path-Time graph(S-T graph) for speed planning
 */
class PathTimeGraph {
 private:
  std::pair<double, double> time_range_;
  std::pair<double, double> path_range_;
  std::vector<STBoundary>   st_boundaries_;

  double path_length_;
  double total_time_;

  const double Default_Path_Width_        = 1.0 * CAR_WIDTH;  // m
  const double Trajectory_Time_Resolution = 0.05;             // Temporarily set
  double       current_speed_;

 public:
  PathTimeGraph() {
    time_range_  = std::make_pair(0.0, 0.0);
    path_range_  = std::make_pair(0.0, 0.0);
    path_length_ = 0;
    total_time_  = 0;
  }

  // Manually add st_boundaries for test use
  void SetStBoundaries(const std::vector<STBoundary>& st_boundaries) {
    st_boundaries_ = st_boundaries;
  }

  double path_length() const { return path_length_; }

  double total_time() const { return total_time_; }

  std::vector<STBoundary>& st_boundaries() { return st_boundaries_; }

  PathTimeGraph(std::vector<Obstacle>& obstacles, const std::vector<Pose>& path,
                double s_start, double s_end, double t_start, double t_end,
                double current_speed = 0);

  /**
   * @brief Get the location(PathPoint) of obstacle at time t
   * @param obstacle
   * @param t Time
   * @return Location(PathPoint) of obstacle at t
   */
  Pose GetPointAtTime(const Obstacle& obstacle, double t) const;

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
  Box GetDynamicBoundingBox(const Obstacle& obstacle, const Pose& point);

  /**
   * @brief calculate ST boundary of obstacle with given vertices
   * @param vertices
   * @param path
   * @return SL Boundary of obstacle
   */
  SLBoundary ComputeObstacleSLBoundary(const std::vector<Vec>&  vertices,
                                       const std::vector<Pose>& path);

  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
      double start_time, double end_time, double trajectory_time_resolution);

  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
    const double t) const;
 private:
  /**
   * @brief Set up obstacles on the s-t graph and get a series of st_boundaries.
   *        Both static obstacles and dynamic obstacles are included.
   * @param obstacles Obstalces list
   * @param path The given path
   */
  void SetupObstacles(std::vector<Obstacle>&   obstacles,
                      const std::vector<Pose>& path);

  void SetDynamicObstacle(Obstacle& obstacle, const std::vector<Pose>& path);

  void SetStaticObstacle(Obstacle& obstacle, const std::vector<Pose>& path);

};

}  // namespace TiEV
