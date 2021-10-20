#include <iostream>

#include "collision_check.h"
#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void FreeDriving::enter(Control& control) {
  entry_time = getTimeStamp();
  cout << "entry Free Driving..." << endl;
}

void FreeDriving::update(FullControl& control) {
  cout << "Free Driving update..." << endl;
  MapManager* map_manager = MapManager::getInstance();
  map_manager->updateRefPath();
  map_manager->updatePlanningMap(MapManager::LaneLineBlockType::NO_BLOCK);
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
    if (p.backward || !p.in_map() ||
        point2LineDis(p, map.boundary_line[0]) < 0 ||
        point2LineDis(p, map.boundary_line[1]) > 0) {
      flag = false;
      break;
    }

  if (flag && !speed_path_list.empty())
    control.changeTo<NormalDriving>();
  else if (speed_path_list.empty() && getTimeStamp() - entry_time > 3e6 &&
           map_manager->getMaintainedPath(map.nav_info).size() < 10)
    control.changeTo<Exploration>();
  else if (getTimeStamp() - entry_time > 5e6)
    control.changeTo<SemiLaneFreeDriving>();
}
}  // namespace TiEV
