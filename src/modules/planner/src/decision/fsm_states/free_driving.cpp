#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void FreeDriving::enter(Control& control) {
    cout << "entry Free Driving..." << endl;
}

void FreeDriving::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
