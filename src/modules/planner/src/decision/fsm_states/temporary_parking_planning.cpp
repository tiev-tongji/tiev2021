#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void TemporaryParkingPlanning::enter(Control& control) {
  cout << "entry Temporary Parking Planning..." << endl;
  entry_time   = getTimeStamp();
  limited_time = 10 * 1e6;
}

void TemporaryParkingPlanning::update(FullControl& control) {
  cout << "Temporary Parking Planning update..." << endl;
  MapManager& map_manager = MapManager::getInstance();
  map_manager.updateRefPath();
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  auto&        map        = map_manager.getMap();
  vector<Pose> start_path = map_manager.getStartMaintainedPath();
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
  if (!maintained_path.empty() &&
      ((maintained_path.front().backward && map.nav_info.current_speed > 0.2)/* ||
       !collision(maintained_path, map.planning_dis_map)*/)) {
    return;
  }
  map_manager.maintainPath(map.nav_info, result_path);
  entry_time               = getTimeStamp();
  const auto& current_pose = map.nav_info.car_pose;

  if (point2PointDis(current_pose, target) <= 2.1 &&
      current_pose.cosDeltaAngle(target) > cos(M_PI / 3)) {
    control.changeTo<TemporaryStop>();
  }
}  // namespace TiEV
}  // namespace TiEV
