#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void Exploration::enter(Control& control) {
    cout << "entry Exploration..." << endl;
}

void Exploration::update(FullControl& control) {
    cout << "Exploration update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    map_manager->updateRefPath();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::NO_BLOCK);
    vector<Pose>      start_path   = map_manager->getStartMaintainedPath();
    vector<Pose>      lane_targets = map_manager->getLaneTargets();
    Map&              map          = map_manager->getMap();
    vector<SpeedPath> speed_path_list;
    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), true, map.lidar_dis_map, map.planning_dis_map, start_path, lane_targets, map.nav_info.current_speed,
                                           speed_path_list);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath(map.nav_info, map.best_path.path);

    if(!speed_path_list.empty())
        control.changeTo<FreeDriving>();
    else {
        vector<Pose> explore_targets = map_manager->getExplorationTargets();
        PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), true, map.lidar_dis_map, map.planning_dis_map, start_path, lane_targets,
                                               map.nav_info.current_speed, speed_path_list);
        map_manager->selectBestPath(speed_path_list);
        map_manager->maintainPath(map.nav_info, map.best_path.path);
    }
}
}  // namespace TiEV
