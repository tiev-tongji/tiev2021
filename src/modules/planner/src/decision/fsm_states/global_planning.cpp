#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void GlobalPlanning::enter(Control& control) {
    cout << "entry Global Planning..." << endl;
}

void GlobalPlanning::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
    cout << "Global Planning update..." << endl;
    control.changeTo<NormalDriving>();
    map_manager->updateRefPath();
}
}  // namespace TiEV
