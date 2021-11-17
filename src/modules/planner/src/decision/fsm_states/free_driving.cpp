#include <iostream>

#include "collision_check.h"
#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void FreeDriving::enter(Control& control) {
  // record the car position
  NavInfo nav_info;
  MessageManager::getInstance().getNavInfo(nav_info);
  previous_pose = nav_info.car_pose;
  entry_time    = getTimeStamp();
  limited_time  = 30e6;  // 30s
  cout << "entry Free Driving..." << endl;
}

void FreeDriving::update(FullControl& control) {
  cout << "Free Driving update..." << endl;
  MapManager& map_manager = MapManager::getInstance();
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::ALL_BLOCK);
  vector<Pose> start_path = map_manager.getStartMaintainedPath();
  Map&         map        = map_manager.getMap();

  std::vector<Pose> result_path;

  const bool plan_to_target = PathPlanner::getInstance().runPathPlanner(
      map.nav_info, map_manager.getLaneCenterDecision(map),
      map.dynamic_obj_list, map_manager.getCurrentMapSpeed(), true,
      map.lidar_dis_map, map.planning_dis_map, start_path, Pose(0, 0, 0),
      &result_path);

  // LOG(INFO) << "Plan to target:" << plan_to_target;
  map_manager.maintainPath(map.nav_info, result_path);
  if (plan_to_target && duration_time() > limited_time) {
    control.changeTo<NormalDriving>();
    return;
  }
  if (duration_time() > limited_time) {
    // when timeout, is the car drive far more than 10m?
    const double dx = previous_pose.utm_position.utm_x -
                      map.nav_info.car_pose.utm_position.utm_x;
    const double dy = previous_pose.utm_position.utm_y -
                      map.nav_info.car_pose.utm_position.utm_y;
    if (dx * dx + dy * dy < 10 * 10) {
      control.changeTo<GlobalReplanning>();
    } else {
      previous_pose = map.nav_info.car_pose;
      entry_time    = getTimeStamp();
    }
  }
}
}  // namespace TiEV
