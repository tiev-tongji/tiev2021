#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void UTurn::enter(Control& control) {
    cout << "entry UTurn..." << endl;
    entry_time              = getTimeStamp();
    MapManager* map_manager = MapManager::getInstance();
    map_manager->maintained_uturn_target.clear();
}

void UTurn::update(FullControl& control) {
    cout << "UTurn update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    map_manager->updateRefPath();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::NO_BLOCK);
    vector<Pose> start_path    = map_manager->getStartMaintainedPath();
    Map&         map           = map_manager->getMap();
    vector<Pose> uturn_targets = map_manager->getUTurnTargets();
    if(uturn_targets.empty()) return;
    vector<SpeedPath> speed_path_list;
    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), true, map.lidar_dis_map, map.planning_dis_map, start_path, uturn_targets,
                                           map.nav_info.current_speed, speed_path_list);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath(map.nav_info, map.best_path.path);
    vector<HDMapPoint> forward_ref_path = map_manager->getForwardRefPath();
    if(map_manager->carInRoad() && !forward_ref_path.empty() && forward_ref_path.front().cosDeltaAngle(map.nav_info.car_pose) > cos(PI / 3)) control.changeTo<NormalDriving>();
    if(getTimeStamp() - entry_time > 20e6) control.changeTo<FreeDriving>();
}
}  // namespace TiEV
