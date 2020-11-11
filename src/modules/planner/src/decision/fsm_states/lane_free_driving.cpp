#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void LaneFreeDriving::enter(Control& control) {
    cout << "entry Lane Free Driving..." << endl;
}

void LaneFreeDriving::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
