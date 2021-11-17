#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void ReplaceParkingPath::enter(Control& control) {
  cout << "entry Replace Parking Path..." << endl;
}

void ReplaceParkingPath::update(FullControl& control) {
  cout << "Replace Parking Path update ..." << endl;
  MapManager& map_manager = MapManager::getInstance();
  Map&        map         = map_manager.getMap();
  // TODO: path
  if (map.forward_ref_path.size() >= 6) control.changeTo<SeekParkingSpot>();
}
}  // namespace TiEV
