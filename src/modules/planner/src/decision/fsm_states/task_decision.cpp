#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void TaskDecision::enter(Control& control) {
    cout << "entry Exploration..." << endl;
}

void TaskDecision::update(FullControl& control) {
    MapManager* map_manager = MapManager::getInstance();
}
}  // namespace TiEV
