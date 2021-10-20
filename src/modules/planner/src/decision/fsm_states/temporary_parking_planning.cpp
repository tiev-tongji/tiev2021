#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void TemporaryParkingPlanning::enter(Control& control) {
  cout << "entry Temporary Parking Planning..." << endl;
}

void TemporaryParkingPlanning::update(FullControl& control) {
  cout << "Temporary Parking Planning update..." << endl;
  MapManager* map_manager = MapManager::getInstance();
  map_manager->updateRefPath();
  map_manager->updatePlanningMap(MapManager::LaneLineBlockType::NO_BLOCK);
  auto&        map        = map_manager->getMap();
  vector<Pose> start_path = map_manager->getStartMaintainedPath();
  Pose         target     = map_manager->getTemporaryParkingTarget();
  if (target.x == 0 && target.y == 0 && target.ang == 0) {
    control.changeTo<NormalDriving>();
    return;
  }
  std::vector<Pose> result_path;
  PathPlanner::getInstance()->runPathPlanner(
      map.nav_info, map.ref_path, map.dynamic_obj_list,
      map_manager->getCurrentMapSpeed(), true, map.lidar_dis_map,
      map.planning_dis_map, start_path, target, &result_path);
  const auto maintained_path = map_manager->getMaintainedPath(map.nav_info);
  if (!maintained_path.empty() && maintained_path.front().backward &&
      map.nav_info.current_speed > 0.2) {
    return;
  }
  map_manager->maintainPath(map.nav_info, result_path);
  const auto& current_pose = map.nav_info.car_pose;

  // if (point2PointDis(current_pose, target) <= 2.1 &&
  //     fabs(current_pose.cosDeltaAngle(target)) > cos(PI / 3)) {
  //   control.changeTo<TemporaryStop>();
  // }
}
}  // namespace TiEV
