#include <iostream>
#include <fstream>
#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"


namespace TiEV {
using namespace std; void Tracking::enter(Control& control) { 
  entry_time = getTimeStamp();
  cout << "entry Tracking..." << endl;
}

void Tracking::update(FullControl& control) {
  if ((getTimeStamp() - entry_time) < 50e3) return;
  LOG(INFO) << "Tracking update..." << endl;
  MapManager& map_manager      = MapManager::getInstance();
  MessageManager& mesg_manager = MessageManager::getInstance();
  NavInfo cur_nav;
  if(mesg_manager.getNavInfo(cur_nav) == 0){
    cerr<<"mesg is unable to get navinfo"<<endl;
  }
  auto cur_car_pos = cur_nav.car_pose;

  auto&       decision_context = DecisionContext::getInstance();
  // remove the dynamic object
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  map_manager.updateRefPath();
  vector<Pose> tracking_path;
  auto         ref_path = map_manager.getForwardRefPath();
  if (ref_path.empty()) {
    LOG(WARNING) << "ref_path empty!";
    return;
  }

  bool                    first_backward = ref_path.front().backward;
  std::vector<HDMapPoint> first_part;
  std::vector<HDMapPoint> second_part;
  bool                    traverse_first = true;
  for (const auto& p : ref_path) {
    if (traverse_first && p.backward == first_backward) {
      if (p.passed) continue;
      first_part.push_back(p);
    } else if (p.backward != first_backward) {
      traverse_first = false;
      second_part.push_back(p);
    } else {
      break;
    }
  }
  if (first_part.size() < 2 && !second_part.empty()) ref_path = second_part;

  double init_s                      = ref_path[0].s;
  bool   is_testing_backward_driving = false;
  for (const auto& p : ref_path) {
    Pose tmp(p.x, p.y, p.ang, p.k, p.v, p.a, p.s, p.t, p.backward,
             p.utm_position);
    tmp.passed = p.passed;
    if (is_testing_backward_driving) {
      if (p.v < 0) {
        tmp.backward = true;
        tmp.v        = 2;
      }
    }
    tracking_path.push_back(tmp);
  }
  if (is_testing_backward_driving) {
    decision_context.setSpeedLimitMPS(2);
  } else {
    decision_context.setSpeedLimitMPS(map_manager.getCurrentMapSpeed());
  }

  




  std::cout <<" tracking path siz is " << tracking_path.size() << std::endl;
  decision_context.setMaintainedPath(tracking_path);

}
}  // namespace TiEV
