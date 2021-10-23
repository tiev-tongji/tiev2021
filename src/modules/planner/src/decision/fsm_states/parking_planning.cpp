#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void ParkingPlanning::enter(Control& control) {
  cout << "entry Parking Planning..." << endl;
}

void ParkingPlanning::update(FullControl& control) {
  cout << "Parking Planning update..." << endl;
  MapManager* map_manager = MapManager::getInstance();
  map_manager->updateRefPath();
  map_manager->updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  vector<Pose> start_path = map_manager->getStartMaintainedPath();
  map_manager->maintainParkingSpots();
  vector<Pose>      targets = map_manager->getParkingSpotTarget();
  Map&              map     = map_manager->getMap();
  vector<SpeedPath> speed_path_list;
  // PathPlanner::getInstance()->runPathPlanner(
  //     map.ref_path, map.dynamic_obj_list, map_manager->getCurrentMapSpeed(),
  //     true, map.lidar_dis_map, map.planning_dis_map, start_path, targets,
  //     map.nav_info.current_speed, speed_path_list);
  map_manager->selectBestPath(speed_path_list);
  map_manager->maintainPath(map.nav_info, map.best_path.path);

  if (false) control.changeTo<Stop>();  // TODO
}
}  // namespace TiEV
