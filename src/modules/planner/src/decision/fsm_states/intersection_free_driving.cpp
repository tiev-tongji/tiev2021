#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void IntersectionFreeDriving::enter(Control& control) {
    cout << "entry Intersection Free Driving..." << endl;
}

void IntersectionFreeDriving::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
