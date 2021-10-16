#include <iostream>

#include "collision_check.h"
#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void LaneFreeDriving::enter(Control& control) {
  entry_time = getTimeStamp();
  cout << "entry Lane Free Driving..." << endl;
}

void LaneFreeDriving::update(FullControl& control) {
  cout << "Lane Free Driving update..." << endl;
  MapManager* map_manager = MapManager::getInstance();
  map_manager->updateRefPath();
  map_manager->updatePlanningMap(MapManager::LaneLineBlockType::ALL_BLOCK);
  vector<Pose>      start_path = map_manager->getStartMaintainedPath();
  vector<Pose>      targets    = map_manager->getLaneTargets();
  Map&              map        = map_manager->getMap();
  vector<SpeedPath> speed_path_list;

  // PathPlanner::getInstance()->runPathPlanner(
  //     map.ref_path, map.dynamic_obj_list, map_manager->getCurrentMapSpeed(),
  //     true, map.lidar_dis_map, map.planning_dis_map, start_path, targets,
  //     map.nav_info.current_speed, speed_path_list);
  map_manager->selectBestPath(speed_path_list);
  map_manager->maintainPath(map.nav_info, map.best_path.path);
  bool flag = true;
  for (const auto& p : map.best_path.path)
    if (p.backward) {
      flag = false;
      break;
    }
  if (flag && !speed_path_list.empty())
    control.changeTo<NormalDriving>();
  else if (speed_path_list.empty() && (getTimeStamp() - entry_time > 3e6))
    control.changeTo<SemiLaneFreeDriving>();
  else if (!speed_path_list.empty())
    entry_time = getTimeStamp();
}
}  // namespace TiEV
