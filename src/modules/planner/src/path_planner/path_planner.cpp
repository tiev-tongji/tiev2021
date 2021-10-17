#include "path_planner.h"

#include <cmath>
#include <cstring>

#include "collision_check.h"
#include "log.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "look_up_tables/reeds_shepp_table/reeds_shepp.h"
#include "splines/Splines.h"
#include "tiev_utils.h"
#include "tievlog.h"

using namespace std;

namespace TiEV {

PathPlanner::PathPlanner()
    : distance_table_rs(
          new DistanceTable(Config::getInstance()->rs_distance_table_path)),
      distance_table_dubins(
          new DistanceTable(Config::getInstance()->dubins_distance_table_path)),
      config(Config::getInstance()),
      view_controller(MessageManager::getInstance()) {}

void PathPlanner::runPathPlanner(const NavInfo&                 nav_info,
                                 const std::vector<HDMapPoint>& ref_path,
                                 const DynamicObjList&          dynamic_objs,
                                 double max_speed, bool reverse,
                                 const double abs_safe_map[MAX_ROW][MAX_COL],
                                 const double lane_safe_map[MAX_ROW][MAX_COL],
                                 const vector<Pose>& start_maintained_path,
                                 const Pose&         target,
                                 std::vector<Pose>*  result) {
  setReferencePath(ref_path);
  setAbsSafeMap(abs_safe_map);
  setLaneSafeMap(lane_safe_map);
  setVelocityLimit(max_speed);
  setStartMaintainedPath(start_maintained_path);
  setTarget(target);
  setBackwardEnabled(reverse);
  setNavInfo(nav_info);
  setDynamicObjList(dynamic_objs);
  plan(result);
}

void PathPlanner::setStartMaintainedPath(
    const vector<Pose>& start_maintained_path) {
  this->start_maintained_path.clear();
  int start_idx = -1;
  for (int i = start_maintained_path.size() - 1; i >= 0; --i) {
    const auto& p = start_maintained_path[i];
    for (const auto& k : k_list) {
      if (fabs(p.k - k) < 1e-8) {
        start_idx = i;
        break;
      }
    }
    if (start_idx >= 0) break;
  }
  this->start_maintained_path = start_maintained_path;
  if (start_idx < 0) {
    this->start_maintained_path.clear();
  } else {
    this->start_maintained_path.resize(start_idx + 1);
  }
  for (auto& p : this->start_maintained_path) {
    p.v = velocity_limit;
  }
  if (!this->start_maintained_path.empty())
    start_pose = this->start_maintained_path.back();
  if (!start_pose.in_map() || this->start_maintained_path.empty()) {
    start_pose = Pose(CAR_CEN_ROW, CAR_CEN_COL, PI);
  }
}

void PathPlanner::setTarget(const Pose& target) { target_pose = target; }

void PathPlanner::setBackwardEnabled(bool enabled) {
  backward_enabled = enabled;
}

void PathPlanner::setVelocityLimit(double speed) { velocity_limit = speed; }

void PathPlanner::setReferencePath(const std::vector<HDMapPoint>& _ref_path) {
  ref_path = _ref_path;
}

void PathPlanner::setAbsSafeMap(const double map[MAX_ROW][MAX_COL]) {
  memcpy(this->abs_safe_map, map, sizeof(this->abs_safe_map));
}

void PathPlanner::setLaneSafeMap(const double map[MAX_ROW][MAX_COL]) {
  memcpy(this->lane_safe_map, map, sizeof(this->lane_safe_map));
}

void PathPlanner::setDynamicObjList(const DynamicObjList& dynamic_obj_list) {
  if (!dynamic_obj_list.detected) {
    this->dynamic_obj_list.detected = false;
    this->dynamic_obj_list.dynamic_obj_list.clear();
  } else {
    this->dynamic_obj_list = dynamic_obj_list;
  }
  // MAYBE: coordinate conversion
}

void PathPlanner::setNavInfo(const NavInfo& nav_info_) { nav_info = nav_info_; }

void PathPlanner::plan(std::vector<Pose>* result) {
  bool planning_to_target = false;
  start_pose.v            = nav_info.current_speed;
  std::vector<astate> result_path;
  // chose the clothoid primitives
  clothoid_base_primitives.prepare(backward_enabled);
  // chose a specific path planning algrithom
  if (!ref_path.empty() && target_pose.x == 0 && target_pose.y == 0 &&
      target_pose.ang == 0) {
    // no target but have reference path
    astate start_state(start_pose.x, start_pose.y, start_pose.ang, 0,
                       start_pose.k, start_pose.backward);
    result_path = tiev_planner.plan(
        dynamic_obj_list, ref_path, start_state, nav_info.current_speed,
        backward_enabled, abs_safe_map, lane_safe_map,
        config->plan_time_limit_ms * 1000, &clothoid_base_primitives);
  } else if (target_pose.x != 0 && target_pose.y != 0 && target_pose.ang != 0) {
    // planning to target
    planning_to_target = true;
  } else {
    LOG(WARNING) << "No reference path and target! to plan";
    return;
  }
  // contruct the whole path
  if (!start_maintained_path.empty()) {
    start_maintained_path.pop_back();
  }
  result->clear();
  result->reserve(start_maintained_path.size() + result_path.size());
  result->insert(result->end(), start_maintained_path.begin(),
                 start_maintained_path.end());
  double offset_s =
      start_maintained_path.empty() ? 0.0 : start_maintained_path.back().s;
  for (const auto& state : result_path) {
    Pose p;
    p.x        = state.x;
    p.y        = state.y;
    p.ang      = state.a;
    p.s        = state.s + offset_s;
    p.k        = state.curvature;
    p.backward = state.is_backward;
    p.v        = velocity_limit;
    p.updateGlobalCoordinate(nav_info.car_pose);
    result->push_back(p);
  }
  if (result_path.empty()) result->clear();
  // sen visualization data
  if (planning_to_target) view_controller->setTarget(target_pose);
  view_controller->setStartPoint(start_pose);
  view_controller->setSafeMap(lane_safe_map);
  view_controller->setPath(*result);
}

}  // namespace TiEV
