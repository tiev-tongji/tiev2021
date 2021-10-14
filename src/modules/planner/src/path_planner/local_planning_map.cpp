#include <opencv2/opencv.hpp>

#include "collision_check.h"
#include "log.h"
#include "math.h"
#include "path_planner.h"
#include "tievlog.h"

namespace TiEV {

#define MULTILINE(SEGMENT) \
  do {                     \
    SEGMENT                \
  } while (false)
#define BINARY_BRANCH(FLAG, SEGMENT)           \
  MULTILINE(                                   \
      if (FLAG) {                              \
        constexpr bool _is_flag_true_ = true;  \
        SEGMENT                                \
      } else {                                 \
        constexpr bool _is_flag_true_ = false; \
        SEGMENT                                \
      })
#define FLAG                  _is_flag_true_
#define IF_FLAG_THEN(SEGMENT) MULTILINE(if constexpr (FLAG){SEGMENT})

constexpr double len(int a, int b) { return sqrt(a * a + b * b); }

void PathPlanner::local_planning_map::prepare(
    const std::vector<HDMapPoint>& _ref_path, const astate& _target,
    double (*_abs_safe_map)[MAX_COL], double (*_lane_safe_map)[MAX_COL],
    bool _backward_enabled) {
  ref_path         = _ref_path;
  target           = _target;
  abs_safe_map     = _abs_safe_map;
  lane_safe_map    = _lane_safe_map;
  backward_enabled = _backward_enabled;

  // when there is no target
  // if (target.x == 0 && target.y == 0 && !ref_path.empty()) {
  if (!ref_path.empty()) {
    // cv::namedWindow("offset_map");
    // cv::Mat dis_img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
    // resize the ref_path to get the first one path
    auto new_size = ref_path.size();
    for (int i = 0; i < ref_path.size(); ++i) {
      const auto& p = ref_path[i];
      if (p.s < 0) continue;
      if (p.x <= 1 || p.x > MAX_ROW - 2 || p.y <= 1 || p.y >= MAX_COL - 2) {
        new_size = i + 1;
        break;
      }
    }
    ref_path.resize(new_size);
    // calculate the ref path end arrival area
    const auto& ref_end_p = ref_path.back();
    // left bound and right bound point
    left_bound_p = ref_end_p.getLateralPose(
        ref_end_p.lane_width * (ref_end_p.lane_num - ref_end_p.lane_seq + 0.5));
    right_bound_p = ref_end_p.getLateralPose(ref_end_p.lane_width *
                                             (ref_end_p.lane_seq - 0.5));
    // get the lane center for each ref path p
    for (auto& p : ref_path) {
      if (!p.neighbors.empty()) continue;
      for (int i = 0; i < p.lane_num; ++i) {
        p.neighbors.push_back(
            p.getLateralPose(p.lane_width * (i - p.lane_seq + 1)));
      }
    }
    // offset the center point to avoid bostacles
    std::vector<std::pair<Point2d, double>> offset_list;

    constexpr double offset_decent = 0.05;  // m
    for (auto it = ref_path.rbegin(); it != ref_path.rend(); it++) {
      auto&                              ref_p = *it;
      std::vector<pair<Point2d, double>> new_offset_list;
      for (int i = 0; i < ref_p.neighbors.size(); ++i) {
        auto&   pi         = ref_p.neighbors[i];
        double  offset_dis = 0.0;
        Point2d origin_p(pi.x, pi.y);
        // dis_img.at<cv::Vec3b>(int(origin_p.x), int(origin_p.y)) =
        //     cv::Vec3b(0, 0, 255);
        // find the closest pre point's offset
        double min_dis = 1e8;
        for (const auto& off_p : offset_list) {
          const double dis =
              sqrDistance(off_p.first.x, off_p.first.y, pi.x, pi.y);
          if (dis < min_dis) {
            min_dis    = dis;
            offset_dis = off_p.second;
          }
        }
        // offset this pi by decent offset_dis
        const auto& offset_after_decent = [&](const double offdis) {
          if (offdis > 0) {
            return std::max(offdis - offset_decent, 0.0);
          } else {
            return std::min(offdis + offset_decent, 0.0);
          }
        };
        offset_dis = offset_after_decent(offset_dis);

        const auto& tmp_off_point = pi.getLateralPose(offset_dis);
        // is this offset safe?
        if (is_lane_crashed(tmp_off_point.x, tmp_off_point.y)) {
          // if the center_p is crash, can it offset to neibor?
          bool safe_offset = false;
          // find the left closest obstatle free center
          for (int j = i + 1; j < ref_p.neighbors.size(); ++j) {
            const auto& pj = ref_p.neighbors[j];
            if (!is_lane_crashed(pj.x, pj.y)) {
              offset_dis =
                  euclideanDistance(pj.x, pj.y, pi.x, pi.y) * GRID_RESOLUTION;
              safe_offset = true;
              break;
            }
          }
          // find the right closest obstatle free center
          for (int j = i - 1; j >= 0; --j) {
            const auto& pj = ref_p.neighbors[j];
            if (!is_lane_crashed(pj.x, pj.y)) {
              Point2d      pij_vec(pj.x - pi.x, pj.y - pi.y);
              const double o_dis =
                  pi.getDirection().cross(pij_vec) * GRID_RESOLUTION;
              if (fabs(o_dis) < offset_dis) {
                offset_dis  = o_dis;
                safe_offset = true;
                break;
              }
            }
          }
          // if the road is all crashed
          if (!safe_offset) {
            offset_dis = (ref_p.neighbors.back() - pi).len() * GRID_RESOLUTION +
                         ref_p.lane_width;
          }
        }
        // offset the pi
        pi.offset(offset_dis);
        new_offset_list.push_back(std::make_pair(origin_p, offset_dis));
      }
      offset_list = new_offset_list;
      // LOG(WARNING) << "---at ref_p:" << ref_p.s << "---";
      // for (const auto& p : offset_list) {
      //   LOG(INFO) << p.first << " offset=" << p.second;
      // }
    }
    // for (const auto& ref_p : ref_path) {
    //   for (const auto& p : ref_p.neighbors) {
    //     if (!is_in_map(int(p.x), int(p.y))) continue;
    //     dis_img.at<cv::Vec3b>(int(p.x), int(p.y)) = cv::Vec3b(255, 100, 0);
    //   }
    // }
    // cv::imshow("offset_map", dis_img);
    // cv::waitKey();
  } else {
    calculate_2d_distance_map();
  }
}

bool PathPlanner::local_planning_map::is_abs_crashed(int x, int y) const {
  return abs_safe_map[x][y] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION;
}

bool PathPlanner::local_planning_map::is_abs_crashed(
    const astate& state) const {
  return collision(state.x, state.y, state.a, abs_safe_map, 0.0);
}

bool PathPlanner::local_planning_map::is_abs_crashed(primitive& prim) const {
  for (const auto& sta : prim.get_states()) {
    if (is_abs_crashed(sta)) return true;
  }
  return false;
}

bool PathPlanner::local_planning_map::is_lane_crashed(int x, int y) const {
  return lane_safe_map[x][y] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION;
}

bool PathPlanner::local_planning_map::is_lane_crashed(
    const astate& state) const {
  return collision(state.x, state.y, state.a, lane_safe_map, 0.5);
}

bool PathPlanner::local_planning_map::is_lane_crashed(primitive& prim) const {
  for (const auto& sta : prim.get_states()) {
    if (is_lane_crashed(sta)) return true;
  }
  return false;
}

double PathPlanner::local_planning_map::get_heuristic(const astate& state,
                                                      bool can_reverse) const {
  double heuristic = 0;
  // along the ref_path, the heuristic is small
  double      min_distance = 1e8;
  HDMapPoint  ref_near_p;
  double      end_s   = 0;
  const auto& sqr_dis = [](const double x0, const double y0, const double x1,
                           const double y1) {
    return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1);
  };
  for (const auto& p : ref_path) {
    double dis = sqr_dis(p.x, p.y, state.x, state.y);
    if (dis < min_distance) {
      min_distance = dis;
      ref_near_p   = p;
    }
    end_s = p.s;
  }
  // calculate the ref_path heuristic for this state
  heuristic += 2 * (end_s - ref_near_p.s);  // guide forward along the ref
  // path guide close to ref path
  // close to lane center
  min_distance = 1e8;
  for (const auto& p : ref_near_p.neighbors) {
    double center_dis = sqr_dis(p.x, p.y, state.x, state.y);
    if (center_dis < min_distance) min_distance = center_dis;
  }
  heuristic += 0.03 * (min_distance);
  // guide heading close to ref path
  heuristic += 10 * (1 - cos(fabs(state.a - ref_near_p.ang)));
  if (!ref_path.empty()) {
    // when pilot with ref_path, more dis_to lane_center heuristic
    // heuristic += 10 * (min_distance);
    return heuristic;
  }
  // if the state is near to target
  // constexpr double target_effect_r = 20 / GRID_RESOLUTION;  // 10m
  // min_distance                     = 1e8;
  // HDMapPoint ref_target_p;
  // for (const auto& p : ref_path) {
  //   double dis = (p.x - target.x) * (p.x - target.x) +
  //                (p.y - target.y) * (p.y - target.y);
  //   if (dis < min_distance) {
  //     min_distance = dis;
  //     ref_target_p = p;
  //   }
  // }
  // const double s_to_target = fabs(ref_target_p.s - ref_near_p.s);
  // if (s_to_target < target_effect_r) {
  //   heuristic -=
  //       std::max(5 * (target_effect_r -
  //                     euclideanDistance(state.x, state.y, target.x,
  //                     target.y)),
  //                0.0);
  // }
  // return heuristic;
  // calculate the target heuristic
  const int x_idx              = (int)state.x;
  const int y_idx              = (int)state.y;
  double    astar_2d_distance  = astar_2d_distance_map[x_idx][y_idx];
  double    rs_dubins_distance = 0;
  double    q0[3]              = {state.x, state.y, state.a};
  double    q1[3]              = {target.x, target.y, target.a};
  if (can_reverse) {
    rs_dubins_distance = getInstance()->distance_table_rs->getDistance(q0, q1);
  } else {
    rs_dubins_distance =
        getInstance()->distance_table_dubins->getDistance(q0, q1);
  }
  // heuristic += 20 * astar_2d_distance;
  heuristic += 20 * std::max(astar_2d_distance, rs_dubins_distance);
  return heuristic;
}

int PathPlanner::local_planning_map::try_get_target_index(
    primitive& primitive) const {
  if (!ref_path.empty()) {
    const auto& states = primitive.get_states();
    for (int i = 0, size = states.size(); i < size; ++i)
      if (is_target(states[i])) return i;
  }
  const auto& start_state = primitive.get_start_state();
  double      dis =
      max(fabs(start_state.x - target.x), fabs(start_state.y - target.y)) *
      GRID_RESOLUTION;

  if (dis <= primitive.get_length()) {
    const auto& states = primitive.get_states();
    for (int i = 0, size = states.size(); i < size; ++i)
      if (is_target(states[i])) return i;
  }

  return -1;
}

bool PathPlanner::local_planning_map::is_in_map(const astate& state) const {
  return is_in_map((int)round(state.x), (int)round(state.y));
}

bool PathPlanner::local_planning_map::is_in_map(int row_idx,
                                                int col_idx) const {
  return row_idx >= 0 && row_idx < MAX_ROW && col_idx >= 0 && col_idx < MAX_COL;
}

bool PathPlanner::local_planning_map::is_target(const astate& state) const {
  // arrive the ref path end
  if (!ref_path.empty()) {                 // and no target
    constexpr double end_area_length = 5;  //  grid
    return euclideanDistance(state.x, state.y, left_bound_p.x, left_bound_p.y) +
               euclideanDistance(state.x, state.y, right_bound_p.x,
                                 right_bound_p.y) <
           ref_path.back().lane_width / GRID_RESOLUTION *
                   ref_path.back().lane_num +
               end_area_length;
  }
  // arrive target
  constexpr double dx = 5;
  constexpr double dy = 5;
  constexpr double da = 15 / 180.0 * M_PI;
  if (fabs(state.x - target.x) > dx) return false;
  if (fabs(state.y - target.y) > dy) return false;
  double dangle = wrap_angle_0_2_PI(state.a - target.a);
  dangle        = min(dangle, 2 * M_PI - dangle);
  return dangle <= da;
}

void PathPlanner::local_planning_map::calculate_2d_distance_map() {
  const int    dx[]  = {0, 0, -1, 1, 1, 1, -1, -1};
  const int    dy[]  = {-1, 1, 0, 0, 1, -1, 1, -1};
  const double dis[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      astar_2d_distance_map[r][c] = 1e8;
    }
  }
  queue<pair<int, int>> obj_que;
  obj_que.push(std::make_pair(int(target.x), int(target.y)));
  astar_2d_distance_map[int(target.x)][int(target.y)] = 0.0;
  while (!obj_que.empty()) {
    const int x = obj_que.front().first;
    const int y = obj_que.front().second;
    obj_que.pop();
    for (int i = 0; i < 8; ++i) {
      const int tx = x + dx[i], ty = y + dy[i];
      if (Point2d(tx, ty).in_map() && !is_lane_crashed(tx, ty) &&
          astar_2d_distance_map[tx][ty] >
              astar_2d_distance_map[x][y] + dis[i]) {
        astar_2d_distance_map[tx][ty] = astar_2d_distance_map[x][y] + dis[i];
        obj_que.push(make_pair(tx, ty));
      }
    }
  }
  // cv::namedWindow("astar_dis_map");
  // cv::Mat dis_img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {0, 0, 0});
  // for (int r = 0; r < MAX_ROW; ++r) {
  //   for (int c = 0; c < MAX_COL; ++c) {
  //     dis_img.at<cv::Vec3b>(r, c) =
  //         cv::Vec3b(0, 0, int(astar_2d_distance_map[r][c]) % 255);
  //   }
  // }
  // cv::imshow("astar_dis_map", dis_img);
  // cv::waitKey();
}

double
PathPlanner::local_planning_map::get_minimum_distance_from_map_boundaries(
    const astate& state) const {
  constexpr double row_2 = (MAX_ROW / 2.0);
  constexpr double col_2 = (MAX_COL / 2.0);
  return min(row_2 - fabs(state.x - row_2), col_2 - fabs(state.y - col_2));
}
}  // namespace TiEV