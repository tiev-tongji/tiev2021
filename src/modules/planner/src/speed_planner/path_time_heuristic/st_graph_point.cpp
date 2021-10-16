#include "st_graph_point.h"

namespace TiEV {

uint32_t StGraphPoint::index_s() const { return index_s_; }

uint32_t StGraphPoint::index_t() const { return index_t_; }

const STPoint& StGraphPoint::st_point() const { return st_point_; }

const StGraphPoint* StGraphPoint::pre_point() const { return pre_point_; }

double StGraphPoint::reference_cost() const { return reference_cost_; }

double StGraphPoint::obstacle_cost() const { return obstacle_cost_; }

double StGraphPoint::total_cost() const { return total_cost_; }

void StGraphPoint::Init(const uint32_t index_t, const uint32_t index_s,
                        const STPoint& st_point) {
  index_t_  = index_t;
  index_s_  = index_s;
  st_point_ = st_point;
}

void StGraphPoint::SetReferenceCost(const double reference_cost) {
  reference_cost_ = reference_cost;
}

void StGraphPoint::SetObstacleCost(const double obs_cost) {
  obstacle_cost_ = obs_cost;
}

void StGraphPoint::SetTotalCost(const double total_cost) {
  total_cost_ = total_cost;
}

void StGraphPoint::SetPrePoint(const StGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}
}  // namespace TiEV