#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void BackTracking::enter(Control& control) {
    cout << "entry BackTracking..." << endl;
}

void BackTracking::update(FullControl& control) {
    cout << "BackTracking update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    map_manager->updateRefPath();
    Map&         map = map_manager->getMap();
    vector<Pose> back_tracking_path;
    for(const auto& p : map.ref_path) {
        if(p.s > 0) break;
        back_tracking_path.emplace_back(p.x, p.y, p.ang, p.k, p.v, p.a, -p.s, p.t, true);
    }
    reverse(back_tracking_path.begin(), back_tracking_path.end());
    map_manager->maintainPath(map.nav_info, back_tracking_path);
}
}  // namespace TiEV
