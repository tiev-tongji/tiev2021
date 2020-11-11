#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void ParkingPlanning::enter(Control& control) {
    cout << "entry Exploration..." << endl;
}

void ParkingPlanning::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
