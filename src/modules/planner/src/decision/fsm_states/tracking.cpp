#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void Tracking::enter(Control& control) {
  entry_time = getTimeStamp();
  cout << "entry Tracking..." << endl;
}

void Tracking::update(FullControl& control) {
  if ((getTimeStamp() - entry_time) < 50e3) return;
  LOG(INFO) << "Tracking update..." << endl;
  MapManager& map_manager      = MapManager::getInstance();
  auto&       decision_context = DecisionContext::getInstance();
  // remove the dynamic object
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  map_manager.updateRefPath();
  vector<Pose> tracking_path;
  const auto&  ref_path = map_manager.getForwardRefPath();
  if (ref_path.empty()) {
    LOG(WARNING) << "ref_path empty!";
    return;
  }
  double init_s = ref_path[0].s;
  for (const auto& p : ref_path) {
    Pose tmp(p.x, p.y, p.ang, p.k, p.v, p.a, p.s, p.t, p.backward,
             p.utm_position);
    tracking_path.push_back(tmp);
  }
  decision_context.setSpeedLimitMPS(map_manager.getCurrentMapSpeed());
  decision_context.setMaintainedPath(tracking_path);
  entry_time = getTimeStamp();
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
}
}  // namespace TiEV
