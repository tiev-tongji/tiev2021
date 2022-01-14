#include <iostream>

#include "map_manager.h"
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
  const auto map = map_manager.getMap();
  const auto start_path = map_manager.getStartMaintainedPath();
  map_manager.updateRefPath();
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  Pose         target     = map_manager.getTemporaryParkingTarget();
  if (!map_manager.allowParking(target, map.ref_path)) {
    control.changeTo<NormalDriving>();
    return;
  }
  std::vector<Pose> result_path;
  PathPlanner::getInstance().runPathPlanner(
      map.nav_info, map.ref_path, map.dynamic_obj_list,
      map_manager.getCurrentMapSpeed(), true, map.lidar_dis_map,
      map.planning_dis_map, start_path, target, &result_path);

  const auto maintained_path = map_manager.getMaintainedPath(map.nav_info);
  if (!maintained_path.empty() && !collision(maintained_path, map.planning_dis_map)) {
    return;
  }
  decision_context.setSpeedLimitMPS(map_manager.getCurrentMapSpeed());
  if (!result_path.empty()) {
    decision_context.setMaintainedPath(result_path);
  }
  decision_context.updatePlannerInfo(map.dynamic_obj_list.dynamic_obj_list);

  entry_time               = getTimeStamp();
  const auto& current_pose = map.nav_info.car_pose;

  if (point2PointDis(current_pose, target) <= 2.1 &&
      current_pose.cosDeltaAngle(target) > cos(M_PI / 3)) {
    control.changeTo<TemporaryStop>();
  }
}  // namespace TiEV
}  // namespace TiEV
