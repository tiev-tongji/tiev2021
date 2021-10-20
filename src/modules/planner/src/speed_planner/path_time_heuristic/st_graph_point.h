#pragma once

#include <stdint-gcc.h>

#include <limits>

#include "st_point.h"

namespace TiEV {

/**
 * @class
 * @brief Point(cell) in gridded path-time graph
 */
class StGraphPoint {
 private:
  STPoint             st_point_;
  const StGraphPoint* pre_point_ = nullptr;
  uint32_t            index_s_   = 0;
  uint32_t            index_t_   = 0;

  double reference_cost_ = 0.0;
  double obstacle_cost_  = 0.0;
  double total_cost_     = std::numeric_limits<double>::infinity();

 public:
  StGraphPoint() = default;

  uint32_t index_s() const;
  uint32_t index_t() const;

  const STPoint&      st_point() const;
  const StGraphPoint* pre_point() const;

  double reference_cost() const;
  double obstacle_cost() const;
  double total_cost() const;

  void Init(const uint32_t index_t, const uint32_t index_s,
            const STPoint& st_point);

  void SetReferenceCost(const double reference_cost);

  void SetObstacleCost(const double obs_cost);

  void SetTotalCost(const double total_cost);

  void SetPrePoint(const StGraphPoint& pre_point);
};
}  // namespace TiEV