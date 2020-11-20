#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void Stop::enter(Control& control) {
    cout << "entry Stop..." << endl;
}

void Stop::update(FullControl& control) {
    // TODO: shutdown
    MapManager*  map_manager = MapManager::getInstance();
    Map&         map         = map_manager->getMap();
    vector<Pose> stop_path;
    stop_path.push_back(map.nav_info.car_pose);
    map_manager->maintainPath(map.nav_info, stop_path);
    // MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
