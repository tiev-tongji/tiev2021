#include <iostream>

#include "map_manager.h"
#include "path_matcher.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void TemporaryParkingPlanning::enter(Control& control) {
  cout << "entry Temporary Parking Planning..." << endl;
  std::vector<Pose> empty_path;
  auto&             decision_context = DecisionContext::getInstance();
  decision_context.setMaintainedPath(empty_path);
  entry_time   = getTimeStamp();
  limited_time = 10 * 1e6;
}

void TemporaryParkingPlanning::update(FullControl& control) {
  LOG(INFO) << "Temporary Parking Driving update...";
  MapManager& map_manager      = MapManager::getInstance();
  auto&       decision_context = DecisionContext::getInstance();
  const auto  map              = map_manager.getMap();
  map_manager.updateRefPath();
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::SEMI_BLOCK);
  Pose target = map_manager.getTemporaryParkingTarget();
  // if (!map_manager.allowParking(target, map.ref_path)) {
  //   control.changeTo<NormalDriving>();
  //   return;
  // }
  std::vector<Pose> result_path;
  std::vector<Pose> start_path;  // empty start path
  PathPlanner::getInstance().runPathPlanner(
      map.nav_info, map.ref_path, map.dynamic_obj_list,
      map_manager.getCurrentMapSpeed(), true, map.lidar_dis_map,
      map.planning_dis_map, start_path, target, &result_path);

  decision_context.setSpeedLimitMPS(
      std::max(1.0, map.nav_info.current_speed - 0.05));
  decision_context.updatePlannerInfo(map.dynamic_obj_list.dynamic_obj_list);

  const auto& maintained_path =
      decision_context.getMaintainedPath(map.nav_info);
  const auto& current_pose = map.nav_info.car_pose;
  sub_target.updateLocalCoordinate(current_pose);

  auto is_target_reached = [&](Pose p, Pose target) {
    if (point2PointDis(p, target) <= 2.1 &&
        p.deltaAngle(target) * 180 / PI < 5) {
      return true;
    }
    return false;
  };

  // check whether to substiture maintain path
  bool   change_maintainpath = false;
  double dis =
      fabs(PathMatcher::MatchToPath(maintained_path, map.nav_info.car_pose)
               .signed_dis);
  if (is_target_reached(current_pose, sub_target) ||
      dis > 0.3 / GRID_RESOLUTION ||
      collision(maintained_path, map.planning_dis_map) ||
      decision_context.getMovementInSeconds(5) < 1) {
    change_maintainpath = true;
  }

  // substitute maintain path
  if (!result_path.empty() && change_maintainpath) {
    int               seg_id = 1;
    std::vector<Pose> first_part;
    std::vector<Pose> second_part;
    for (int i = 1; i < result_path.size(); ++i) {
      const auto& p   = result_path[i];
      const auto& p_1 = result_path[i - 1];
      if (p_1.backward != p.backward) {
        seg_id++;
      }
      if (i == 1) first_part.push_back(p_1);
      if (seg_id == 1) {
        first_part.push_back(p);
      }
      if (seg_id == 2) {
        second_part.push_back(p);
      }
      if (seg_id > 2) break;
    }
    if (first_part.back().s - first_part.front().s < 0.4) {
      decision_context.setMaintainedPath(second_part);
      sub_target = second_part.back();
    } else {
      decision_context.setMaintainedPath(first_part);
      sub_target = first_part.back();
    }
  }

  if (is_target_reached(current_pose, target)) {
    control.changeTo<TemporaryStop>();
  } else {
    if (getTimeStamp() - entry_time < 50e3) {
      usleep(50e3 - getTimeStamp() + entry_time);
    }
    entry_time = getTimeStamp();
  }
}
}  // namespace TiEV
