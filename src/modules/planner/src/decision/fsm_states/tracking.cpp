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
    vector<Pose> tracking_path;
    for(const auto& p : map.forward_ref_path)
        tracking_path.emplace_back(p.x, p.y, p.ang, p.k, p.v, p.a, p.s, p.t, p.backward);
    map_manager->maintainPath(map.nav_info, tracking_path);
}
}  // namespace TiEV
