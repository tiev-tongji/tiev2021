#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void NormalDriving::enter(Control& control) {
  LOG(INFO) << "entry Normal Driving...";
  entry_time   = getTimeStamp();
  limited_time = 3e6;
}

void NormalDriving::update(FullControl& control) {
  LOG(INFO) << "Normal Driving update...";
  MapManager* map_manager = MapManager::getInstance();
  const auto  start1      = getTimeStamp();
  map_manager->updateRefPath();
  LOG(INFO) << "updte refpath:" << (getTimeStamp() - start1) * 1e-3 << "ms";

  const auto start2 = getTimeStamp();
  map_manager->updatePlanningMap(MapManager::LaneLineBlockType::ALL_BLOCK);
  LOG(INFO) << "updte plnanner map:" << (getTimeStamp() - start2) * 1e-3
            << "ms";

  const auto start_path = map_manager->getStartMaintainedPath();
  // const auto targets    = map_manager->getLaneTargets();
  const auto& map = map_manager->getMap();

  vector<SpeedPath> speed_path_list;
  const auto        start3 = getTimeStamp();
  std::vector<Pose> result_path;
  PathPlanner::getInstance()->runPathPlanner(
      map.ref_path, map.dynamic_obj_list, map_manager->getCurrentMapSpeed(),
      false, map.lidar_dis_map, map.planning_dis_map, start_path,
      {Pose(0, 0, 0)}, map.nav_info.current_speed, &result_path);
  LOG(INFO) << "planning time:" << (getTimeStamp() - start3) * 1e-3 << "ms";

  // map_manager->selectBestPath(speed_path_list);
  const auto maintained_path = map_manager->getMaintainedPath(map.nav_info);
  if (maintained_path.empty() ||
      maintained_path.back().s < map.nav_info.current_speed * 5 + 5)
    map_manager->maintainPath(map.nav_info, result_path);
  if (speed_path_list.empty() && duration_time() > limited_time) {
    // control.changeTo<LaneFreeDriving>();
  } else if (!speed_path_list.empty()) {
    entry_time = getTimeStamp();
  }
}

}  // namespace TiEV
