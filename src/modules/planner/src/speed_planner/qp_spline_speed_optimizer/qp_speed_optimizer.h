#pragma once

#include "osqp_spline_solver.h"
#include "path_time_graph.h"
#include "pose.h"
#include "qp_st_config.h"
#include "speed_data.h"
#include "speed_limit.h"

namespace TiEV {

class QpSpeedOptimizer {
 private:
  QpStConfig          qp_st_config_;  // QP st configuration
  Pose                init_point_;    // initial states
  Pose                target_point_;
  Spline1d            splines_;                       // QP result
  double              t_knots_resolution_ = 0.0;      // t knots resolution
  std::vector<double> t_knots_;                       // knots
  double              t_evaluated_resolution_ = 0.0;  // evaluated t resolution
  std::vector<double> t_evaluated_;                   // evaluated points
  SpeedLimit          speed_limit_;

 public:
  QpSpeedOptimizer(const Pose& init_point, const Pose& target_point,
                   const SpeedLimit& speed_limit);

  bool Process(PathTimeGraph&   st_graph_data,
               const SpeedData& reference_dp_speed_points);

  // QP result getter
  const Spline1d& splines() { return splines_; }

  // retrieve speed data
  bool RetrieveSpeedData(SpeedData& speed_data);

  const SpeedLimit& speed_limit() { return speed_limit_; }

 private:
  void Init();

  // Add st graph constraint
  bool AddConstraint(const Pose& init_point, const double speed_limit,
                     const std::vector<STBoundary>&   boundaries,
                     const std::pair<double, double>& accel_bound);

  // Add objective function
  bool AddKernel(const std::vector<STBoundary>& boundaries,
                 const double                   speed_limit);

  // solve
  bool Solve();

  // Extract upper lower bound for constraint
  bool GetSConstraintByTime(const std::vector<STBoundary> boundaries,
                            const double time, const double total_path_length,
                            double* const s_upper_bound,
                            double* const s_lower_bound) const;
};
}  // namespace TiEV
