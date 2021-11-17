#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void SafeDriving::enter(Control& control) {
  expired_time    = getTimeStamp() + 10 * 1000 * 1000;
  entry_time      = getTimeStamp() + 10 * 1000 * 1000;
  last_block_time = getTimeStamp();
  cout << "entry Safe Driving..." << endl;
}

void SafeDriving::update(FullControl& control) {
  LOG(INFO) << "Safe Driving update...";
  MapManager& map_manager = MapManager::getInstance();
  // remove the dynamic object
  map_manager.updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  const auto map = map_manager.getMap();

  bool       back_ward  = map.nav_info.current_speed < 3 ? true : false;
  const auto start_path = map_manager.getStartMaintainedPath();

  // RoadDirection direction =
  // map_manager.getForwardRefPath().front().direction;
  RoadDirection direction = map_manager.getCurrentRoadDirection();
  if (map.traffic_light.detected) {
    if ((direction == RoadDirection::RIGHT && !map.traffic_light.right) ||
        (direction == RoadDirection::LEFT && !map.traffic_light.left) ||
        (direction == RoadDirection::STRAIGHT && !map.traffic_light.straight)) {
      map_manager.blockStopLine();
    }
  }

  HDMapPoint stop_point, crosswalk_end_point;
  for (int i = 0; i < map.forward_ref_path.size(); ++i) {
    const HDMapPoint& p = map.forward_ref_path[i];
    if (p.event == HDMapEvent::ENTRY_INTERSECTION ||
        p.event == HDMapEvent::EXIT_INTERSECTION) {
      stop_point = p;
      // using several points to estimate the ang of the stop_point
      int start = max(0, i - 2);
      int end   = min((int)(map.forward_ref_path.size() - 1), i + 2);
      if (end - start <= 0) break;
      double sum_ang = 0;
      for (int j = start; j <= end; ++j) {
        sum_ang += map.forward_ref_path[j].ang;
      }
      stop_point.ang = sum_ang / (end - start + 1);
      break;
    };
  }

  // ensure that stop_point exit
  if (stop_point.x != 0 && stop_point.y != 0) {
    // counterclockwise rotate -stop_point.ang is
    // equivalent to clockwise rotate stop_point.ang
    double rotate_ang = -stop_point.ang;
    // regard stop_point as origin
    // regard stop_point.ang direction as x axis
    // rotate (crosswalk_width, 0) from stop_point frame to grid map frame
    double crosswalk_width  = 4 / GRID_RESOLUTION;
    crosswalk_end_point     = stop_point;
    crosswalk_end_point.x   = stop_point.x + crosswalk_width * cos(rotate_ang);
    crosswalk_end_point.y   = stop_point.y - crosswalk_width * sin(rotate_ang);
    crosswalk_end_point.ang = stop_point.ang;
    vector<Pose> box_half_crosswalk;
    vector<Pose> box_whole_crosswalk;
    // lateral distance of corners
    // positive for left point. negative for right point
    double lat_dis_left, lat_dis_right;
    lat_dis_right = -stop_point.lane_width * (stop_point.lane_seq - 1 + 0.5);
    lat_dis_left  = stop_point.lane_width *
                   (stop_point.lane_num - stop_point.lane_seq + 0.5);
    // four corners should be in order. clock or counter-clock wise.
    box_half_crosswalk.push_back(stop_point.getLateralPose(lat_dis_right));
    box_half_crosswalk.push_back(stop_point.getLateralPose(lat_dis_left));
    box_half_crosswalk.push_back(
        crosswalk_end_point.getLateralPose(lat_dis_left));
    box_half_crosswalk.push_back(
        crosswalk_end_point.getLateralPose(lat_dis_right));

    lat_dis_right =
        -stop_point.lane_width * (stop_point.lane_seq - 1 + 0.5 + 2);
    lat_dis_left = 2 * stop_point.lane_width *
                   (stop_point.lane_num - stop_point.lane_seq + 0.5);
    box_whole_crosswalk.push_back(stop_point.getLateralPose(lat_dis_right));
    box_whole_crosswalk.push_back(stop_point.getLateralPose(lat_dis_left));
    box_whole_crosswalk.push_back(
        crosswalk_end_point.getLateralPose(lat_dis_left));
    box_whole_crosswalk.push_back(
        crosswalk_end_point.getLateralPose(lat_dis_right));

    // the position of these two corners depends on the previous order
    Point2d corner_lr = (box_half_crosswalk[0] + box_half_crosswalk[3]) / 2;
    Point2d corner_ll = (box_half_crosswalk[1] + box_half_crosswalk[2]) / 2;
    Point2d vector_lr, vector_ll;  // vector from obj to two corners above
    bool    block_stop_line = false;
    for (const auto& obj : map.dynamic_obj_list.dynamic_obj_list) {
      if (obj.type != ObjectType::PEDESTRIAN) continue;
      if (obj.path.empty()) continue;
      vector_lr         = corner_lr - obj.path.front();
      vector_ll         = corner_ll - obj.path.front();
      Point2d front_end = obj.path.back() - obj.path.front();
      bool    in_inner_box =
          map_manager.pnbox(obj.path.front(), box_half_crosswalk);
      bool in_outer_box =
          map_manager.pnbox(obj.path.front(), box_whole_crosswalk);
      double costheta1 = front_end.cosDeltaAngle(vector_ll);
      double costheta2 = front_end.cosDeltaAngle(vector_lr);
      if (in_inner_box) block_stop_line = true;
      if (in_outer_box && front_end.len() > 1 / GRID_RESOLUTION &&
          (costheta1 > 0 || costheta2 > 0)) {
        block_stop_line = true;
      }
    }
    // only when the free_time of crosswalk is greater than a certain number
    // ego vehicle can drive through the crosswalk
    time_t free_time = 0;
    if (!block_stop_line)
      free_time = getTimeStamp() - last_block_time;
    else
      last_block_time = getTimeStamp();  // update last_block_time

    if (free_time < 1e6) map_manager.blockStopLine();
  }

  // // TODO: Pedestrian
  // map_manager.addPedestrian(map.dynamic_obj_list,
  //                            map_manager.getForwardRefPath());

  std::vector<Pose> result_path;
  PathPlanner::getInstance().runPathPlanner(
      map.nav_info, map_manager.getLaneCenterDecision(map),
      map.dynamic_obj_list, map_manager.getCurrentMapSpeed(), back_ward,
      map.lidar_dis_map, map.planning_dis_map, start_path, Pose(0, 0, 0),
      &result_path);

  const auto maintained_path = map_manager.getMaintainedPath(map.nav_info);
  if (!maintained_path.empty() && maintained_path.front().backward &&
      map.nav_info.current_speed > 0.2) {
    return;
  }
  map_manager.maintainPath(map.nav_info, result_path);
}
}  // namespace TiEV
