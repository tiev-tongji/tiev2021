#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void NormalDriving::enter(Control& control) {
    cout << "entry Normal Driving..." << endl;
}

void NormalDriving::update(FullControl& control) {
    cout << "Normal Driving update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    map_manager->updateRefPath();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::ALL_BLOCK);
    vector<Pose>      start_path = map_manager->getStartMaintainedPath();
    vector<Pose>      targets    = map_manager->getLaneTargets();
    Map&              map        = map_manager->getMap();
    vector<SpeedPath> speed_path_list;
    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, 10, false, map.lidar_dis_map, map.planning_dis_map, start_path, targets, map.nav_info.current_speed, speed_path_list);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath();
}

}  // namespace TiEV
