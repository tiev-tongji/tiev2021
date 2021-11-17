#ifndef DECISION_CONTEXT_H
#define DECISION_CONTEXT_H
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <vector>

#include "tiev_class.h"

namespace TiEV {
struct PlannerInfo {
  // planner info of each iteration
};

// magic static
class DecisionContext {
 public:
  // getter and setter
  const std::vector<DynamicObj>& getPedestrianDecision() const;
  const std::vector<DynamicObj>& getTrafficLightDecision() const;

  const std::vector<DynamicObj> getStaticObsDecision() const;
  const std::vector<DynamicObj> getDynamicList() const;

  const std::vector<Pose> getMaintainedPath() const;

  const std::queue<PlannerInfo> getPlannerHitory() const;

  const double getSpeedLimitKPH() const;
  const double getSpeedLimitMPS() const;
  const double getCarSpeedMPS() const;

  void setPedestrianDecision(
      const std::vector<DynamicObj>& pedestrian_decision_result);
  void setTrafficLightDecision(
      const std::vector<DynamicObj>& traffic_light_decision_result);

  void setMaintainedPath(const std::vector<Pose> path);

  void setPlannerInfo(const PlannerInfo& planner_info);

  void setSpeedLimitMPS(const double speed_limit_mps);

 private:
  // decision results
  std::vector<DynamicObj> _pedestrian_decision_result;
  std::vector<DynamicObj> _traffic_light_decision_result;

  double _speed_limit_mps;

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
