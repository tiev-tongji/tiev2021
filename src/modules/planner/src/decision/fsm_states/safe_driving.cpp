#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void SafeDriving::enter(Control& control) {
    expired_time = getTimeStamp() + 60 * 1000 * 1000;
    cout << "entry Safe Driving..." << endl;
}

void SafeDriving::update(FullControl& control) {
    cout << "Safe Driving update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    map_manager->updateRefPath();
    map_manager->updatePlanningMap(MapManager::LaneLineBlockType::ALL_BLOCK, true);
    vector<Pose>      start_path = map_manager->getStartMaintainedPath();
    vector<Pose>      targets    = map_manager->getLaneTargets();
    vector<SpeedPath> speed_path_list;
    // 红绿灯
    RoadDirection direction = map.forward_ref_path.front().direction;
    if(direction == RoadDirection::RIGHT && !map.traffic_light.right || direction == RoadDirection::LEFT && !map.traffic_light.left
       || direction == RoadDirection::STRAIGHT && !map.traffic_light.straight) {
        map_manager->blockStopLine();
    }
    map_manager->avoidPedestrian();

    PathPlanner::getInstance()->runPlanner(map.dynamic_obj_list, map_manager->getCurrentMapSpeed(), false, map.lidar_dis_map, map.planning_dis_map, start_path, targets, map.nav_info.current_speed,
                                           speed_path_list);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath(map.nav_info, map.best_path.path);

    if(map.nav_info.current_speed < 0.05) {
        time_t now_time = getTimeStamp();
        if(now_time >= expired_time) control.changeTo<IntersectionFreeDriving>();
    }
    else {
        expired_time = getTimeStamp() + 60 * 1000 * 1000;
    }
}
}  // namespace TiEV
