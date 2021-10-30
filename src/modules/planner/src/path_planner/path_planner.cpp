#include "path_planner.h"

#include <cmath>
#include <cstring>

#include "collision_check.h"
#include "log.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "look_up_tables/reeds_shepp_table/reeds_shepp.h"
#include "opencv2/opencv.hpp"
#include "path_smoother.h"
#include "splines/Splines.h"
#include "tiev_utils.h"
#include "tievlog.h"

using namespace std;

namespace TiEV {

PathPlanner::PathPlanner()
    : distance_table_rs(
          new DistanceTable(Config::getInstance()->rs_distance_table_path)),
      distance_table_dubins(
          new DistanceTable(Config::getInstance()->dubins_distance_table_path)),
      config(Config::getInstance()),
      view_controller(MessageManager::getInstance()) {}

bool PathPlanner::runPathPlanner(const NavInfo&                 nav_info,
                                 const std::vector<HDMapPoint>& ref_path,
                                 const DynamicObjList&          dynamic_objs,
                                 double max_speed, bool reverse,
                                 const double abs_safe_map[MAX_ROW][MAX_COL],
                                 const double lane_safe_map[MAX_ROW][MAX_COL],
                                 const vector<Pose>& start_maintained_path,
                                 const Pose&         target,
                                 std::vector<Pose>*  result) {
  setReferencePath(ref_path);
  setAbsSafeMap(abs_safe_map);
  setLaneSafeMap(lane_safe_map);
  setVelocityLimit(max_speed);
  setStartMaintainedPath(start_maintained_path);
  setTarget(target);
  setBackwardEnabled(reverse);
  setNavInfo(nav_info);
  setDynamicObjList(dynamic_objs);
  return plan(result);
}

void PathPlanner::setStartMaintainedPath(
    const vector<Pose>& start_maintained_path) {
  this->start_maintained_path.clear();
  int start_idx = -1;
  for (int i = start_maintained_path.size() - 1; i >= 0; --i) {
    const auto& p = start_maintained_path[i];
    for (const auto& k : k_list) {
      if (fabs(p.k - k) < 1e-8) {
        start_idx = i;
        break;
      }
    }
    if (start_idx >= 0) break;
  }
  this->start_maintained_path = start_maintained_path;
  if (start_idx < 0) {
    this->start_maintained_path.clear();
  } else {
    this->start_maintained_path.resize(start_idx + 1);
  }
  for (auto& p : this->start_maintained_path) {
    p.v = velocity_limit;
  }
  if (!this->start_maintained_path.empty()) {
    start_pose = this->start_maintained_path.back();
    this->start_maintained_path.pop_back();
  }
  if (!start_pose.in_map() || this->start_maintained_path.empty()) {
    start_pose = Pose(CAR_CEN_ROW, CAR_CEN_COL, PI);
  }
}

void PathPlanner::setTarget(const Pose& target) { target_pose = target; }

void PathPlanner::setBackwardEnabled(bool enabled) {
  backward_enabled = enabled;
}

void PathPlanner::setVelocityLimit(double speed) { velocity_limit = speed; }

void PathPlanner::setReferencePath(const std::vector<HDMapPoint>& _ref_path) {
  ref_path = _ref_path;
}

void PathPlanner::setAbsSafeMap(const double map[MAX_ROW][MAX_COL]) {
  memcpy(this->abs_safe_map, map, sizeof(this->abs_safe_map));
}

void PathPlanner::setLaneSafeMap(const double map[MAX_ROW][MAX_COL]) {
  memcpy(this->lane_safe_map, map, sizeof(this->lane_safe_map));
}

void PathPlanner::setDynamicObjList(const DynamicObjList& dynamic_obj_list) {
  if (!dynamic_obj_list.detected) {
    this->dynamic_obj_list.detected = false;
    this->dynamic_obj_list.dynamic_obj_list.clear();
  } else {
    this->dynamic_obj_list = dynamic_obj_list;
  }
  // MAYBE: coordinate conversion
}

void PathPlanner::setNavInfo(const NavInfo& nav_info_) { nav_info = nav_info_; }

bool PathPlanner::plan(std::vector<Pose>* result) {
  bool planning_to_target = false;
  start_pose.v            = nav_info.current_speed;
  std::vector<astate> result_path;
  // a flag stands for plan a path in time
  bool plan_in_time = false;
  // chose a specific path planning algrithom
  astate start_state(start_pose.x, start_pose.y, start_pose.ang, start_pose.s,
                     start_pose.k, start_pose.backward);
  if (!ref_path.empty() && target_pose.x == 0 && target_pose.y == 0 &&
      target_pose.ang == 0) {
    clothoid_base_primitives.prepare(backward_enabled);
    // no target but have reference path
    result_path = tiev_planner.plan(dynamic_obj_list, ref_path, start_state,
                                    nav_info.current_speed, backward_enabled,
                                    abs_safe_map, lane_safe_map,
                                    config->plan_time_limit_ms * 1000,
                                    &clothoid_base_primitives, &plan_in_time);
  } else if (target_pose.x != 0 || target_pose.y != 0 || target_pose.ang != 0) {
    // planning to target
    astate target_state(target_pose.x, target_pose.y, target_pose.ang, 0,
                        target_pose.k, target_pose.backward);
    planning_to_target = true;
    result_path        = astar_planner.plan(
        start_state, target_state, nav_info.current_speed, backward_enabled,
        abs_safe_map, lane_safe_map, config->plan_time_limit_ms * 1000,
        &arc_base_primitives);
    view_controller->setPriorityLane({});
  } else {
    LOG(WARNING) << "No reference path and target to plan !";
    return false;
  }
  // contruct the whole path
  result->clear();
  result->reserve(start_maintained_path.size() + result_path.size());
  result->insert(result->end(), start_maintained_path.begin(),
                 start_maintained_path.end());

  // only smooth path that is long enough
  if (result_path.size() > 5) {
    // smooth the path
    vector<Point2d> path_before_smooth;
    path_before_smooth.reserve(result_path.size());
    for (auto it = result_path.begin(); it != result_path.end(); ++it) {
      path_before_smooth.emplace_back(it->x, it->y);
    }
    PathSmoother ps;
    // make sure the size of path_after_smooth and result_path is equal
    vector<Point2d> path_after_smooth = ps.smoothPath(path_before_smooth);
    // if collision happen, don't substitute the original path
    Point2d p, pp;
    double  ang_tmp;
    bool    smooth_path_crash = false;
    for (int i = 0; i + 1 < path_after_smooth.size(); ++i) {
      p       = path_after_smooth[i];
      pp      = path_after_smooth[i + 1];
      ang_tmp = std::atan2(pp.y - p.y, pp.x - p.x);
      if (collision(p.x, p.y, ang_tmp, lane_safe_map, 0.0)) {
        smooth_path_crash = true;
        break;
      }
    }
    if (!smooth_path_crash) {
      Point2d xim1, xi, xip1;
      double  total_s = 0;
      for (int i = 1; i + 1 < path_after_smooth.size(); ++i) {
        xim1 = path_after_smooth[i - 1];
        xi   = path_after_smooth[i];
        xip1 = path_after_smooth[i + 1];
        total_s += (xi - xim1).len() * GRID_RESOLUTION;
        result_path[i].x         = path_after_smooth[i].x;
        result_path[i].y         = path_after_smooth[i].y;
        result_path[i].a         = std::atan2(xi.y - xim1.y, xi.x - xim1.x);
        result_path[i].curvature = ps.getCurvature(xim1, xi, xip1);
        result_path[i].s         = result_path[0].s + total_s;
      }
      if (result_path.size() > 1) {
        astate p, pp;
        pp                   = result_path[1];
        p                    = result_path[0];
        result_path[0].a     = std::atan2(pp.y - p.y, pp.x - p.x);
        int length           = result_path.size();
        pp                   = result_path[length - 1];
        p                    = result_path[length - 2];
        result_path.back().a = std::atan2(pp.y - p.y, pp.x - p.x);
      }
    }
  }
  // for (int i = 1; i + 1 < result_path.size(); ++i) {
  //   cout << "original k: " << result_path[i].curvature << "\tsmoothed k: "
  //        << ps.getCurvature(path_after_smooth[i - 1], path_after_smooth[i],
  //                           path_after_smooth[i + 1])
  //        << endl;
  // }

// #define VIS_SMOOTHED_PATH
#ifdef VIS_SMOOTHED_PATH
  cv::namedWindow("smoothed_path", cv::WINDOW_KEEPRATIO);
  cv::Mat img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      if (lane_safe_map[r][c] == 0) {
        img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
      }
    }
  }
  for (const auto& p : path_before_smooth) {
    img.at<cv::Vec3b>(lround(p.x), lround(p.y)) = cv::Vec3b(0, 255, 0);
  }
  for (const auto& p : path_after_smooth) {
    img.at<cv::Vec3b>(lround(p.x), lround(p.y)) = cv::Vec3b(0, 0, 255);
  }
  cv::imshow("smoothed_path", img);
  cv::waitKey(0);
#endif

  for (const auto& state : result_path) {
    Pose p;
    p.x        = state.x;
    p.y        = state.y;
    p.ang      = state.a;
    p.s        = state.s;
    p.k        = state.curvature;
    p.backward = state.is_backward;
    p.v        = velocity_limit;
    p.updateGlobalCoordinate(nav_info.car_pose);
    result->push_back(p);
  }
  if (result_path.empty())
    result->clear();
  else if (planning_to_target) {
    plan_in_time = true;
  }
  // sen visualization data
  if (planning_to_target) view_controller->setTarget(target_pose);
  view_controller->setStartPoint(start_pose);
  view_controller->setSafeMap(lane_safe_map);
  view_controller->setPath(*result);
  return plan_in_time;
}

}  // namespace TiEV
