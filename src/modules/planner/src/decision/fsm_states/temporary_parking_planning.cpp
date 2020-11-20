#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void TemporaryParkingPlanning::enter(Control& control) {
    cout << "entry Temporary Parking Planning..." << endl;
}

void TemporaryParkingPlanning::update(FullControl& control) {
    cout << "Temporary Parking Planning update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    auto&       map         = map_manager->getMap();
    map_manager->updateRefPath();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::NO_BLOCK);
    vector<Pose> start_path = map_manager->getStartMaintainedPath();
    vector<Pose> targets    = map_manager->getTemporaryParkingTarget();
    // if(targets.empty()) targets = map_manager->getLaneTargets();
    if(targets.empty()) return;
    vector<SpeedPath> speed_path_list;
    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), true, map.lidar_dis_map, map.planning_dis_map, start_path, targets, map.nav_info.current_speed,
                                           speed_path_list);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath(map.nav_info, map.best_path.path);
    Pose& current_pose = map.nav_info.car_pose;

    if(point2PointDis(current_pose, targets.front()) <= 2.1 && fabs(current_pose.cosDeltaAngle(targets.front())) > cos(PI / 3)) {
        control.changeTo<TemporaryStop>();
    }
}
}  // namespace TiEV
