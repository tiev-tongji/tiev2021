#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void SemiLaneFreeDriving::enter(Control& control) {
    cout << "entry Semi-Lane Free Driving..." << endl;
}

void SemiLaneFreeDriving::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
