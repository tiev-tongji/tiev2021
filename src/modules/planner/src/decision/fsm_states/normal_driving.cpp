#include <iostream>

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
  MapManager* map_manager = MapManager::getInstance();
  // remove the dynamic object
  map_manager->updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  const auto map = map_manager->getMap();

  bool       back_ward  = map.nav_info.current_speed < 3 ? true : false;
  const auto start_path = map_manager->getStartMaintainedPath();

  std::vector<Pose> result_path;
  PathPlanner::getInstance()->runPathPlanner(
      map.nav_info, map_manager->getLaneCenterDecision(map),
      map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), back_ward,
      map.lidar_dis_map, map.planning_dis_map, start_path, Pose(0, 0, 0),
      &result_path);

  const auto maintained_path = map_manager->getMaintainedPath(map.nav_info);
  if (!maintained_path.empty() && maintained_path.front().backward &&
      map.nav_info.current_speed > 0.2) {
    return;
  }
  map_manager->maintainPath(map.nav_info, result_path);
  if (map_manager->allowParking(map_manager->getTemporaryParkingTarget(),
                                map.ref_path)) {
    // when to parking
    control.changeTo<TemporaryParkingPlanning>();
  } else if (map.nav_info.current_speed < 0.1) {
    // the car is top
    if (!maintained_path.empty() && duration_time() > limited_time) {
      control.changeTo<FreeDriving>();
    }
  } else {
    entry_time = getTimeStamp();
  }
}

}  // namespace TiEV
