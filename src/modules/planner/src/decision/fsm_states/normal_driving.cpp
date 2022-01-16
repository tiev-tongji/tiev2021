
#include <iostream>

#include "lattice_planner.h"
#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
void NormalDriving::enter(Control& control) {
  LOG(INFO) << "entry Normal Driving...";
  entry_time   = getTimeStamp();
  limited_time = 5e6;
}

void NormalDriving::update(FullControl& control) {
  LOG(INFO) << "Normal Driving update...";
  MapManager& map_manager      = MapManager::getInstance();
  auto&       decision_context = DecisionContext::getInstance();
  // remove the dynamic object
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::ALL_BLOCK);
  const auto map = map_manager.getMap();

  bool       back_ward  = map.nav_info.current_speed < 0.1 ? true : false;
  const auto start_path = map_manager.getStartMaintainedPath();
  // if we need u-turn, the heading dif weight should be bigger
  const auto& ref_path = map_manager.getForwardRefPath();
  if (!ref_path.empty()) {
    const bool need_reverse = ref_path.front().getDirectionVec().dot(
                                  map.nav_info.car_pose.getDirectionVec()) < 0;
    if (need_reverse) {
      decision_context.setPlanningWeights({1, 0.01, 0.003, 0.001, 5, 1, 2});
    } else {
      decision_context.setPlanningWeights({1, 0.02, 0.2, 0.03, 2.5, 5, 0});
      // decision_context.setPlanningWeights({1, 0.02, 0.03, 0.01, 2.5, 0, 0});
      // a good setting for overtake driving
      // decision_context.setPlanningWeights({1, 0.02, 0.008, 0.06, 2, 1, 5});
    }
  } else {
    return;
  }

  std::vector<Pose> result_path;
  PathPlanner::getInstance().runPathPlanner(
      map.nav_info, map_manager.getLaneCenterDecision(map),
      map.dynamic_obj_list, map_manager.getCurrentMapSpeed(), back_ward,
      map.lidar_dis_map, map.planning_dis_map, start_path, Pose(0, 0, 0),
      &result_path);

  const auto maintained_path     = decision_context.getMaintainedPath();
  bool       change_maintainpath = true;
  if (!maintained_path.empty() && maintained_path.front().backward &&
      map.nav_info.current_speed > 0.2) {
    change_maintainpath = false;
  }
  if (decision_context.getMovementInSeconds(5) < 1) {
    change_maintainpath = true;
  }
  decision_context.setSpeedLimitMPS(map_manager.getCurrentMapSpeed());
  if (!result_path.empty() && change_maintainpath) {
    decision_context.setMaintainedPath(result_path);
  }
  decision_context.updatePlannerInfo(map.dynamic_obj_list.dynamic_obj_list);

  // LOG(INFO) << map_manager.getTemporaryParkingTarget();
  if (map_manager.allowParking(map_manager.getTemporaryParkingTarget(),
                               ref_path)) {
    // when to parking
    control.changeTo<TemporaryParkingPlanning>();
  } else if (false) {  // TODO
    // the car is top
    if (!maintained_path.empty() && duration_time() > limited_time) {
      // control.changeTo<FreeDriving>();
    }
  } else if (decision_context.getCarSpeedMPS() > MIN_OVERTAKE_SPEED) {
    // to overtaking driving
    // control.changeTo<OvertakeDriving>();
  } else {
    if (getTimeStamp() - entry_time < 50e3) {
      usleep(50e3 - getTimeStamp() + entry_time);
    }
    entry_time = getTimeStamp();
  }
}

}  // namespace TiEV
