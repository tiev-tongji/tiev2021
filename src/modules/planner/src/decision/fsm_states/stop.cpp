#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void Stop::enter(Control& control) {
    cout << "entry Stop..." << endl;
}

void Stop::update(FullControl& control) {
    // TODO: shutdown

    // MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
