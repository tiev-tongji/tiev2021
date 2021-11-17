#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void Tracking::enter(Control& control) { cout << "entry Tracking..." << endl; }

void Tracking::update(FullControl& control) {
  cout << "Tracking update..." << endl;
  MapManager& map_manager = MapManager::getInstance();
  map_manager.updateRefPath();
  Map&         map = map_manager.getMap();
  vector<Pose> tracking_path;
  for (const auto& p : map.forward_ref_path)
    tracking_path.emplace_back(p.x, p.y, p.ang, p.k, p.v, p.a, p.s, p.t,
                               p.backward);
  /*
lineInterpolation<Pose>(tracking_path, 1);
for(int i = 1; i < tracking_path.size(); ++i){
  Pose &p = tracking_path[i];
  if(p.s == 0){
      p.ang = tracking_path[i-1].ang;
      p.k = tracking_path[i-1].k;
      p.backward = tracking_path[i-1].backward;
      p.s = tracking_path[i-1].s+(p-tracking_path[i-1]).len()*GRID_RESOLUTION;
   }
}
*/
  map_manager.maintainPath(map.nav_info, tracking_path);
}
}  // namespace TiEV
