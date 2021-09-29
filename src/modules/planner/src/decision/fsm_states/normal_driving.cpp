#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void NormalDriving::enter(Control& control) {
    cout << "entry Normal Driving..." << endl;
    entry_time = getTimeStamp();
}

void NormalDriving::update(FullControl& control) {
    cout << "Normal Driving update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    const auto start1 = getTimeStamp();
    map_manager->updateRefPath();
    std::cout << "updte refpath:" << (getTimeStamp() - start1) * 1e-3 << "ms" << std::endl;
    const auto start2 = getTimeStamp();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::ALL_BLOCK);
        std::cout << "updte plnanner map:" << (getTimeStamp() - start2)*1e-3 << "ms" << std::endl;
    vector<Pose>      start_path = map_manager->getStartMaintainedPath();
    vector<Pose>      targets    = map_manager->getLaneTargets();
    Map&              map        = map_manager->getMap();
    vector<SpeedPath> speed_path_list;
    const auto start3 = getTimeStamp();
    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), false, map.lidar_dis_map, map.planning_dis_map, start_path, targets, map.nav_info.current_speed,
                                           speed_path_list);
    std::cout << "planning time:" << (getTimeStamp() - start3)*1e-3 << "ms" << std::endl;
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath(map.nav_info, map.best_path.path);
    if(speed_path_list.empty() && getTimeStamp() - entry_time > 3e6)
        control.changeTo<LaneFreeDriving>();
    else if(!speed_path_list.empty())
        entry_time = getTimeStamp();
}

}  // namespace TiEV
