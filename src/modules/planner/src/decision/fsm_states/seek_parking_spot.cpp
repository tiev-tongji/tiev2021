#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void SeekParkingSpot::enter(Control& control) {
    cout << "entry Seek Parking Spot..." << endl;
}

void SeekParkingSpot::update(FullControl& control) {
    cout << "Seek Parking Spot update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    if(map.parking_lot_list.detected)
        control.changeTo<ParkingPlanning>();
    else if(map.ref_path.size() < 6)
        control.changeTo<ReplaceParkingPath>();
}
}  // namespace TiEV
