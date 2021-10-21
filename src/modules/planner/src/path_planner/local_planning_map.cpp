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
    const DynamicObjList&          dynamic_obj_list,
    const std::vector<HDMapPoint>& _ref_path, double (*_abs_safe_map)[MAX_COL],
    double (*_lane_safe_map)[MAX_COL], bool _backward_enabled) {
  is_planning_to_target = false;
  ref_path              = _ref_path;
  abs_safe_map          = _abs_safe_map;
  lane_safe_map         = _lane_safe_map;
  backward_enabled      = _backward_enabled;
  // calculate the ref path end arrival area
  const auto& ref_end_p = ref_path.back();
  // left bound and right bound point
  left_bound_p = ref_end_p.getLateralPose(
      ref_end_p.lane_width * (ref_end_p.lane_num - ref_end_p.lane_seq + 0.5));
  right_bound_p = ref_end_p.getLateralPose(ref_end_p.lane_width *
                                           (ref_end_p.lane_seq - 0.5));
}

void PathPlanner::local_planning_map::prepare(const astate& _target,
                                              double (*_abs_safe_map)[MAX_COL],
                                              double (*_lane_safe_map)[MAX_COL],
                                              bool _backward_enabled) {
  is_planning_to_target = true;
  target                = _target;
  abs_safe_map          = _abs_safe_map;
  lane_safe_map         = _lane_safe_map;
  backward_enabled      = _backward_enabled;

  calculate_2d_distance_map();
}

bool PathPlanner::local_planning_map::is_abs_crashed(int x, int y) const {
  if (!is_in_map(x, y)) return false;
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
  if (!is_in_map(x, y)) return false;
  return lane_safe_map[x][y] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION;
}

bool PathPlanner::local_planning_map::is_lane_crashed(
    const astate& state) const {
  return collision(state.x, state.y, state.a, lane_safe_map, 0.0);
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
  if (!is_planning_to_target) {
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
      if (!p.have_priority) continue;
      double center_dis = sqr_dis(p.x, p.y, state.x, state.y);
      if (center_dis < min_distance) min_distance = center_dis;
    }
    heuristic += 0.03 * (min_distance);
    // guide heading close to ref path
    heuristic += 10 * (1 - cos(fabs(state.a - ref_near_p.ang)));
    // guide to away from obstacles
    const double max_obstacle_affect_dis = 2.5;  // m
    heuristic +=
        std::pow(std::min<double>(lane_safe_map[int(state.x)][int(state.y)] *
                                          GRID_RESOLUTION -
                                      max_obstacle_affect_dis,
                                  0.0),
                 2);
    return heuristic;
  }
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
  heuristic += std::max(astar_2d_distance, rs_dubins_distance);
  return heuristic;
}

int PathPlanner::local_planning_map::try_get_target_index(
    primitive& primitive) const {
  if (!is_planning_to_target) {
    const auto& states = primitive.get_states();
    for (int i = 0, size = states.size(); i < size; ++i)
      if (is_target(states[i])) return i;
  } else {
    const auto& start_state = primitive.get_start_state();
    double      dis =
        max(fabs(start_state.x - target.x), fabs(start_state.y - target.y)) *
        GRID_RESOLUTION;

    if (dis <= primitive.get_length()) {
      const auto& states = primitive.get_states();
      for (int i = 0, size = states.size(); i < size; ++i)
        if (is_target(states[i])) return i;
    }
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
  if (!is_planning_to_target) {
    constexpr double end_area_length = 5;  //  grid
    bool             distance_arrive =
        euclideanDistance(state.x, state.y, left_bound_p.x, left_bound_p.y) +
            euclideanDistance(state.x, state.y, right_bound_p.x,
                              right_bound_p.y) <
        ref_path.back().lane_width / GRID_RESOLUTION *
                ref_path.back().lane_num +
            end_area_length;
    bool heading_arrive = std::cos(ref_path.back().ang - state.a) > 0.71;
    return distance_arrive && heading_arrive;
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