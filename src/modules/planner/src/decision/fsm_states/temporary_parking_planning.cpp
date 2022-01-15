#include <iostream>

#include "map_manager.h"
#include "path_matcher.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void TemporaryParkingPlanning::enter(Control& control) {
  cout << "entry Temporary Parking Planning..." << endl;
  entry_time   = getTimeStamp();
  limited_time = 10 * 1e6;
}

void TemporaryParkingPlanning::update(FullControl& control) {
  LOG(INFO) << "Temporary Parking Driving update...";
  MapManager& map_manager      = MapManager::getInstance();
  auto&       decision_context = DecisionContext::getInstance();
  const auto  map              = map_manager.getMap();
  const auto  start_path       = map_manager.getStartMaintainedPath();
  map_manager.updateRefPath();
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::SEMI_BLOCK);
  Pose target = map_manager.getTemporaryParkingTarget();
  if (!map_manager.allowParking(target, map.ref_path)) {
    control.changeTo<NormalDriving>();
    return;
  }
  std::vector<Pose> result_path;
  PathPlanner::getInstance().runPathPlanner(
      map.nav_info, map.ref_path, map.dynamic_obj_list,
      map_manager.getCurrentMapSpeed(), true, map.lidar_dis_map,
      map.planning_dis_map, start_path, target, &result_path);

  decision_context.setSpeedLimitMPS(
      std::min(2.0, map_manager.getCurrentMapSpeed()));
  decision_context.updatePlannerInfo(map.dynamic_obj_list.dynamic_obj_list);
  const auto maintained_path = decision_context.getMaintainedPath(map.nav_info);
  double     dis =
      PathMatcher::MatchToPath(maintained_path, map.nav_info.car_pose).dis;
  if (maintained_path.size() > 5 && dis < 0.5 / GRID_RESOLUTION &&
      (maintained_path.back() - target).len() < 1 &&
      (maintained_path.front() - map.nav_info.car_pose).len() < 1 &&
      !collision(maintained_path, map.planning_dis_map)) {
    if (getTimeStamp() - entry_time < 50e3) {
      usleep(50e3 - getTimeStamp() + entry_time);
    }
    entry_time = getTimeStamp();
    return;
  }
  if (!result_path.empty()) {
    decision_context.setMaintainedPath(result_path);
  }

  const auto& current_pose = map.nav_info.car_pose;
  if (point2PointDis(current_pose, target) <= 2.1 &&
      current_pose.cosDeltaAngle(target) > cos(M_PI / 3)) {
    control.changeTo<TemporaryStop>();
  } else {
    if (getTimeStamp() - entry_time < 50e3) {
      usleep(50e3 - getTimeStamp() + entry_time);
    }
    entry_time = getTimeStamp();
  }
}
}  // namespace TiEV
