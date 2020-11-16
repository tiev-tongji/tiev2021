#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void GlobalReplanning::enter(Control& control) {
    cout << "entry Global Replanning..." << endl;
}

void GlobalReplanning::update(FullControl& control) {
    cout << "Global Replanning update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    // TODO: map_manager->runRouting(, );
    // control.changeTo<FreeDriving>();
    // map_manager->updateRefPath();
}
}  // namespace TiEV
