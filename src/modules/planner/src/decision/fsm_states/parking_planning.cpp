#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void ParkingPlanning::enter(Control& control) {
    cout << "entry Parking Planning..." << endl;
}

void ParkingPlanning::update(FullControl& control) {
    cout << "Parking Planning update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    if(map.nav_info.current_speed < 0.01) control.changeTo<Stop>();
}
}  // namespace TiEV
