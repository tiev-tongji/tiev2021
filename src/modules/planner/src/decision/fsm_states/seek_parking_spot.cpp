#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void SeekParkingSpot::enter(Control& control) {
    cout << "entry Seek Parking Spot..." << endl;
}

void SeekParkingSpot::update(FullControl& control) {
    cout << "Seek Parking Spot update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    map_manager->updateRefPath();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::NO_BLOCK);
    vector<Pose>      start_path      = map_manager->getStartMaintainedPath();
    vector<Pose>      explore_targets = map_manager->getExplorationTargets();
    vector<SpeedPath> speed_path_list;
    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), true, map.lidar_dis_map, map.planning_dis_map, start_path, explore_targets,
                                           map.nav_info.current_speed, speed_path_list);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath(map.nav_info, map.best_path.path);

    if(map.parking_lot_list.detected)
        control.changeTo<ParkingPlanning>();
    else if(map.ref_path.size() < 6)
        control.changeTo<ReplaceParkingPath>();
}
}  // namespace TiEV
