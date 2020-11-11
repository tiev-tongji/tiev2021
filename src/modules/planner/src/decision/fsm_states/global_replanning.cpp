#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void GlobalReplanning::enter(Control& control) {
    cout << "entry Global Replanning..." << endl;
}

void GlobalReplanning::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
