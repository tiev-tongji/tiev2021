#include <iostream>

#include "collision_check.h"
#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void OvertakeDriving::enter(Control& control) {
  cout << "entry Overtake Driving..." << endl;
  entry_time = getTimeStamp();
}

void OvertakeDriving::update(FullControl& control) {
  cout << "Overtake Driving update..." << endl;
  MapManager* map_manager = MapManager::getInstance();
  map_manager->updatePlanningMap(MapManager::DynamicBlockType::NO_BLOCK);
  vector<Pose> start_path = map_manager->getStartMaintainedPath();
  const auto   map        = map_manager->getMap();

  std::vector<Pose> result_path;
  PathPlanner::getInstance()->runPathPlanner(
      map.nav_info, overtakeLaneDecision(map), map.dynamic_obj_list,
      map_manager->getCurrentMapSpeed(), false, map.lidar_dis_map,
      map.planning_dis_map, start_path, Pose(0, 0, 0), &result_path);

  const auto maintained_path = map_manager->getMaintainedPath(map.nav_info);
  map_manager->maintainPath(map.nav_info, result_path);
}

std::vector<HDMapPoint> OvertakeDriving::overtakeLaneDecision(
    const Map& decision_map) {
  auto ref_path = decision_map.ref_path;

  // resize the ref_path to get the first one path
  auto new_size = ref_path.size();
  for (int i = 0; i < ref_path.size(); ++i) {
    const auto& p = ref_path[i];
    if (p.s < 0) continue;
    if (p.s >= 120 || int(p.x) <= 2 || int(p.x) > MAX_ROW - 2 ||
        int(p.y) <= 1 || int(p.y) >= MAX_COL - 2) {
      new_size = i + 1;
      break;
    }
  }
  ref_path.resize(new_size);
  // get the lane center for each ref path p
  for (auto& p : ref_path) {
    if (!p.neighbors.empty() || p.mode == CHANGE) continue;
    for (int i = 0; i < p.lane_num; ++i) {
      const auto& neighbor_p =
          p.getLateralPose(p.lane_width * (i - p.lane_seq + 1));
      p.neighbors.emplace_back(neighbor_p.x, neighbor_p.y, true, 0.0);
    }
    // maybe we could add a oppsite lane center with no priority
  }
  const auto& clash_with_dynamic = [&](const double x, const double y) {
    for (const auto& obj : decision_map.dynamic_obj_list.dynamic_obj_list) {
      Point2d o_p(x, y);
      for (const auto& p : obj.corners) {
        if ((p - o_p).len() * GRID_RESOLUTION < COLLISION_CIRCLE_SMALL_R)
          return true;
      }
    }
    return false;
  };
  // set the center line priority to avoid bostacles
  // forward check the path
  auto last_ref_p_it = ref_path.end();
  for (auto it = ref_path.begin(); it != ref_path.end(); it++) {
    auto& ref_p = *it;
    if (ref_p.s < 0) continue;
    for (int i = 0; i < ref_p.neighbors.size(); ++i) {
      auto& center_p = ref_p.neighbors[i];
      // if the center is far away with the car more than 1.5*lane_width
      if (ref_p.s == 0) {
        if ((center_p - Point2d(CAR_CEN_ROW, CAR_CEN_COL)).len() *
                GRID_RESOLUTION >
            1.5 * ref_p.lane_width) {
          center_p.have_priority                = false;
          center_p.accumulate_dis_with_priority = 0.0;
        }
      }
      // find the closest pre lane center point and set priority by pre
      LaneCenterPoint pre_nearest_center_point;
      double          min_dis = std::numeric_limits<double>::max();
      if (last_ref_p_it != ref_path.end()) {
        for (const auto& last_center_p : last_ref_p_it->neighbors) {
          const double dis = (last_center_p - center_p).len() * GRID_RESOLUTION;
          if (dis < min_dis) {
            min_dis                  = dis;
            pre_nearest_center_point = last_center_p;
          }
        }
      }
      // decide the priority
      if (collision(center_p.x, center_p.y, decision_map.planning_dis_map) ||
          clash_with_dynamic(center_p.x, center_p.y)) {
        center_p.have_priority                = false;
        center_p.accumulate_dis_with_priority = 0.0;
      } else if (last_ref_p_it != ref_path.end()) {
        center_p.have_priority = pre_nearest_center_point.have_priority;
        center_p.accumulate_dis_with_priority =
            pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
      }
    }
    // check if all the lane center have no priority
    bool all_have_no_priority = true;
    for (const auto& cp : ref_p.neighbors) {
      if (cp.have_priority) {
        all_have_no_priority = false;
        break;
      }
    }
    if (all_have_no_priority) {
      // the last_ref_p is the farest position we can go
      break;
    }
    last_ref_p_it = it;
  }
  // process all the rest point
  auto farest_it = last_ref_p_it;
  for (auto it = farest_it; it != ref_path.end(); ++it) {
    auto& ref_p = *it;
    for (int i = 0; i < ref_p.neighbors.size(); ++i) {
      auto& center_p = ref_p.neighbors[i];
      // find the closest pre lane center point and set priority by pre
      LaneCenterPoint pre_nearest_center_point;
      double          min_dis = std::numeric_limits<double>::max();
      if (last_ref_p_it != ref_path.end()) {
        for (const auto& last_center_p : last_ref_p_it->neighbors) {
          const double dis = (last_center_p - center_p).len() * GRID_RESOLUTION;
          if (dis < min_dis) {
            min_dis                  = dis;
            pre_nearest_center_point = last_center_p;
          }
        }
        // decide the priority
        center_p.have_priority = pre_nearest_center_point.have_priority;
        center_p.accumulate_dis_with_priority =
            pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
      }
    }
    last_ref_p_it = it;
  }
  last_ref_p_it = farest_it;
  for (auto it = farest_it; it != ref_path.begin() && it != ref_path.end();
       --it) {
    auto& ref_p = *it;
    for (int i = 0; i < ref_p.neighbors.size(); ++i) {
      auto& center_p = ref_p.neighbors[i];
      // find the closest pre lane center point and set priority by pre
      LaneCenterPoint pre_nearest_center_point;
      double          min_dis = std::numeric_limits<double>::max();
      if (last_ref_p_it != ref_path.begin()) {
        for (const auto& last_center_p : last_ref_p_it->neighbors) {
          const double dis = (last_center_p - center_p).len() * GRID_RESOLUTION;
          if (dis < min_dis) {
            min_dis                  = dis;
            pre_nearest_center_point = last_center_p;
          }
        }
        // decide the priority
        center_p.have_priority = pre_nearest_center_point.have_priority;
        center_p.accumulate_dis_with_priority =
            pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
      }
    }
    last_ref_p_it = it;
  }
  MessageManager::getInstance()->setPriorityLane(ref_path);
  return ref_path;
}
}  // namespace TiEV
