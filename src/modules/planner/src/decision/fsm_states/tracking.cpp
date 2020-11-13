#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void Tracking::enter(Control& control) {
    cout << "entry Tracking..." << endl;
}

void Tracking::update(FullControl& control) {
    cout << "Tracking update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    map_manager->updateRefPath();
    Map&         map = map_manager->getMap();
    vector<Pose> tracking_path(map.forward_ref_path.size());
    for(const auto& p : map.forward_ref_path)
        tracking_path.emplace_back(p.x, p.y, p.ang, p.k, p.v, p.a, p.s, p.t, p.backward);
    vector<SpeedPath> speed_path_list;
    // SpeedPath speed_path = SpeedOptimizer::RunSpeedOptimizer(map.dynamic_obj_list.dynamic_obj_list, path, speed_limits[target_index], speed_paths[target_index].path.back().s);
    // speed_path_list.push_back(speed_path);
    map_manager->selectBestPath(speed_path_list);
    map_manager->maintainPath();
}
}  // namespace TiEV
