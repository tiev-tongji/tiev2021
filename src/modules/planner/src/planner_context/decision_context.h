#ifndef DECISION_CONTEXT_H
#define DECISION_CONTEXT_H
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <vector>

#include "tiev_class.h"

namespace TiEV {
class PlannerInfo {
  // planner info of each iteration
};

struct PlanningWeights {
  // heuristic weight decision results for path planner
  double w1;  // target_dis_w;
  double w2;  // target_sqrdis_w;
  double w3;  // lane_center_dis_w;
  double w4;  // lane_center_sqrdis_w;
  double w5;  // ref_heading_dis_w;
  double w6;  // obstacle_dis_w;
  double w7;  // reverse cost w;
};

// magic static
class DecisionContext {
 public:
  // getter and setter
  const std::vector<DynamicObj>& getPedestrianDecision() const;
  const std::vector<DynamicObj>& getTrafficLightDecision() const;

  const std::vector<DynamicObj> getStaticObsDecision() const;
  const std::vector<DynamicObj> getDynamicList() const;

  const std::vector<Pose> getMaintainedPath() const;  // use new latest nav_info
  const std::vector<Pose> getMaintainedPath(const NavInfo& nav_info) const;

  const std::queue<PlannerInfo> getPlannerHitory() const;

  const double getSpeedLimitKPH() const;
  const double getSpeedLimitMPS() const;
  const double getCarSpeedMPS() const;

  const PlanningWeights& getPlanningWeights() const;

  void setPedestrianDecision(
      const std::vector<DynamicObj>& pedestrian_decision_result);
  void setTrafficLightDecision(
      const std::vector<DynamicObj>& traffic_light_decision_result);

  void setMaintainedPath(const std::vector<Pose> path);

  void setPlannerInfo(const PlannerInfo& planner_info);

  void setSpeedLimitMPS(const double speed_limit_mps);

  void setPlanningWeights(const PlanningWeights& weights);

 private:
  // decision results
  std::vector<DynamicObj> _pedestrian_decision_result;
  std::vector<DynamicObj> _traffic_light_decision_result;

  double          _speed_limit_mps;
  PlanningWeights _weights;

  // some maintained variables
  std::vector<Pose> _maintained_path;

  // the hitory info of planner, max size = 100;
  static constexpr size_t max_buffer_size = 100;
  std::queue<PlannerInfo> _planner_history_buffer;

 public:
  DecisionContext(const DecisionContext&) = delete;
  DecisionContext& operator=(const DecisionContext&) = delete;

  static DecisionContext& getInstance() {
    static DecisionContext decision_context;
    return decision_context;
  }

 private:
  ~DecisionContext() {}
  DecisionContext() {}
  mutable std::shared_mutex pedestrian_mutex;
  mutable std::shared_mutex traffic_light_mutex;
  mutable std::shared_mutex maintained_path_mutex;
  mutable std::shared_mutex planner_info_mutex;
  mutable std::shared_mutex speed_limit_mutex;
};
}  // namespace TiEV
#endif
