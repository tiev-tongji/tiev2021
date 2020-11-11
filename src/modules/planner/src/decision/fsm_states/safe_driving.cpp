#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void SafeDriving::enter(Control& control) {
    cout << "entry Safe Driving..." << endl;
}

void SafeDriving::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
