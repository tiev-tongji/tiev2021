#include <unistd.h>

#include <random>

#include "log.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "math.h"
#include "opencv2/opencv.hpp"
#include "path_planner.h"
#include "tievlog.h"

// #define NO_TIME_LIMIT
// #define VIS_EXPANSION

namespace TiEV {

#ifdef DEBUG_EXPANSION_CALLBACK
PathPlanner::node_expansion_callback node_expanded;
#endif

#ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
PathPlanner::analytic_expansion_callback analytic_expanded;
#endif

const std::vector<PathPlanner::astate>& PathPlanner::TiEVPlanner::plan(
    const DynamicObjList&          dynamic_obj_list,
    const std::vector<HDMapPoint>& ref_path, const astate& _start_state,
    double current_speed, bool _is_backward_enabled,
    double (*_abs_safe_map)[MAX_COL], double (*_lane_safe_map)[MAX_COL],
    time_t _max_duration, const base_primitive_set* _base_primitives,
    bool* plan_in_time) {
#ifdef VIS_EXPANSION
  cv::namedWindow("expansion", cv::WINDOW_KEEPRATIO);
  cv::Mat expansion_img = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, {255, 255, 255});
  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      if (_lane_safe_map[r][c] == 0) {
        expansion_img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
      }
    }
  }
  for (const auto& p : ref_path) {
    expansion_img.at<cv::Vec3b>(lround(p.x), lround(p.y)) =
        cv::Vec3b(0, 255, 0);
  }
#endif
  constexpr double max_dcc = -0.5;  // m/s^2
  start_time               = getTimeStamp();
  dead_line                = start_time + _max_duration;
  iterations               = 0;
  base_primitives          = _base_primitives;

  start_state     = _start_state;
  start_speed_m_s = std::sqrt(
      std::max(2.0 * max_dcc * start_state.s + current_speed * current_speed,
               0.0));  // v = sqrt(2*a*s+ v_0^2)
  is_backward_enabled = _is_backward_enabled;

  memset(node_visited_map, false, sizeof(node_visited_map));
  result.clear();
  analytic_expansion_result.clear();
  primitive_pool.clear();

  // clear node_pool
  while (!node_pool.empty()) node_pool.pop();
  planning_map.prepare(dynamic_obj_list, ref_path, _abs_safe_map,
                       _lane_safe_map, is_backward_enabled);
  // LOG(WARNING) << "start:" << start_state << " target:" << target_state;
#ifdef VIS_EXPANSION
  cv::circle(expansion_img, cv::Point(int(start_state.y), int(start_state.x)),
             3, cv::Scalar(0, 255, 0), -1);
  cv::circle(expansion_img, cv::Point(int(target_state.y), int(target_state.x)),
             3, cv::Scalar(0, 0, 255), -1);
#endif
  LOG(INFO) << "TiEV Planner initialized";

  // push start_state to node pool
  node_pool.push({planning_map.get_heuristic(
                      start_state, start_state.is_backward, start_speed_m_s),
                  start_speed_m_s, 0.0, 0.0, nullptr});

  bool target_reached = false;
  // last_primitive_ptr points to the last primitive
  // in the whole primitive link after target is reached
  // otherwise it is null
  primitive_ptr last_primitive_ptr = nullptr;
  // target_offset is the offset of the most close state to
  // the target in sampled_states of the last primitive.
  int        target_offset         = -1;
  double     max_cos_with_ref_path = -1;
  const auto get_cos_with_ref_path = [&ref_path](const astate& sta) {
    double min_dis = 1e8;
    double cos_val = -1;
    for (const auto& ref_p : ref_path) {
      const double sqr_dis = sqrDistance(ref_p.x, ref_p.y, sta.x, sta.y);
      if (sqr_dis < min_dis) {
        min_dis = sqr_dis;
        cos_val = std::cos(sta.a - ref_p.ang);
      }
    }
    return cos_val;
  };
  while (!(is_time_out() || target_reached || node_pool.empty())) {
    // get current node
    /*
    LOG(WARNING) << "-----node_pool_start------";
    std::vector<node> tmp_pool;
    tmp_pool.reserve(node_pool.size());
    while (!node_pool.empty()) {
      tmp_pool.push_back(node_pool.top());
      node_pool.pop();
    }
    for (const auto& n : tmp_pool) {
      LOG(INFO) << "score = " << n.score << " cost=" << n.cost
                << " heuristic=" << n.score - n.cost;
      node_pool.push(n);
    }
    LOG(WARNING) << "-----node_pool_end------";
    //*/
    node current = node_pool.top();
    node_pool.pop();

    // get current state
    astate                      current_state;
    std::vector<base_primitive> bases;
    // only the first node in the open list has
    // a null current.ptr
    if (current.ptr == nullptr) {
      current_state = _start_state;
    } else {
      current_state = current.ptr->get_end_state();
    }
    const double state_possible_speed = std::sqrt(std::max(
        2.0 * max_dcc * current_state.s + current_speed * current_speed,
        0.0));  // v = sqrt(2*a*s+ v_0^2)
    bases = base_primitives->get_nexts(current_state, state_possible_speed);
    // LOG(WARNING) << "current_state:" << current_state
    //              << " score=" << current.score;

    bool reverse_allowed = is_backward_enabled;

    for (const auto& base : bases) {
      // if this expansion changes the driving direction
      // (backward/forward)
      bool reversed_expansion =
          (base.get_states().front().is_backward != current_state.is_backward);
      primitive& expansion = *(primitive_pool.make(base, current.ptr));
      if (current_speed < 3) {
        const double cos_with_ref =
            get_cos_with_ref_path(expansion.get_end_state());
        if (cos_with_ref > max_cos_with_ref_path) {
          max_cos_with_ref_path = cos_with_ref;
          last_primitive_ptr    = &expansion;
        }
      }
      // LOG(INFO) << "expansion size=" << primitive_pool.size();
      // LOG(INFO) << "is_visited=" << is_visited(expansion.get_end_state());
#ifdef VIS_EXPANSION
      // LOG(INFO) << "---expansion k_step="
      //           << expansion.get_end_state().curvature -
      //                  expansion.get_start_state().curvature
      //           << "---";
      for (const auto& st : expansion.get_states()) {
        // LOG(INFO) << st;
        expansion_img.at<cv::Vec3b>(int(st.x), int(st.y))[1] -= 5;
        expansion_img.at<cv::Vec3b>(int(st.x), int(st.y))[2] -= 5;
      }
      cv::imshow("expansion", expansion_img);
      cv::waitKey();
#endif
      // for (const auto& st : expansion.get_states()) {
      //   LOG(INFO) << st;
      // }
      if (is_visited(expansion.get_end_state())) continue;
      // check if primitive crashed
      // is_crashed does not check if the primitive is completely
      // or partly outside the local map. if the primitive is
      // not crashed but partly outside the local map, we still
      // assume that a sampled state on it might reach the target
      if (!planning_map.is_lane_crashed(expansion)) {
        // primitive is not crashed
        astate end_state = expansion.get_end_state();
        // check if primitive is near target
        target_offset = planning_map.try_get_target_index(expansion);
        // if target has been reached, end the searching
        if (target_offset >= 0) {
          last_primitive_ptr = &expansion;
          target_reached     = true;
          break;
        }
        // if end_state is in the local map, add it to open list
        else if (planning_map.is_in_map(end_state)) {
          // update end_state to history
          visit(end_state);
          double heuristic = planning_map.get_heuristic(
              end_state, reverse_allowed, current_speed);
          double cost_factor = get_cost_factor(current_state, end_state);
          double cost        = current.cost + base.get_length() * cost_factor;
          double dis_after_reverse =
              base.get_length() +
              (reversed_expansion ? 0.0 : current.dis_after_reverse);
          node_pool.push({heuristic + cost, state_possible_speed, cost,
                          dis_after_reverse, &expansion});
        }
      }
    }
  }

  LOG(INFO) << "end searching after " << iterations << " iterations";
  LOG(INFO) << "searching duration: " << (getTimeStamp() - start_time) / 1000
            << " ms";

  if (target_reached) {
    *plan_in_time = true;
  }
  // path stores the reversed states list
  vector<astate> path;

  if (last_primitive_ptr != nullptr) {
    vector<astate> states(last_primitive_ptr->get_states());
    path.insert(path.end(), states.rend() - target_offset - 1, states.rend());

    last_primitive_ptr = last_primitive_ptr->get_parent();
    while (last_primitive_ptr != nullptr) {
      vector<astate> states = last_primitive_ptr->get_states();
      path.insert(path.end(), states.rbegin() + 1, states.rend());
      last_primitive_ptr = last_primitive_ptr->get_parent();
    }
  }

  result.insert(result.begin(), path.rbegin(), path.rend());
  // result.insert(result.end(), analytic_expansion_result.begin(),
  //               analytic_expansion_result.end());
  // } else
  //   log_1("timeout: no result found");
  return result;
}

void PathPlanner::TiEVPlanner::visit(const astate& state) {
  double a       = PathPlanner::wrap_angle_0_2_PI(state.a);
  int    ang_idx = a / (2 * M_PI / ang_num);
  node_visited_map[lround(2 * state.x)][lround(2 * state.y)][ang_idx] = true;
}

bool PathPlanner::TiEVPlanner::is_visited(const astate& state) {
  double a       = PathPlanner::wrap_angle_0_2_PI(state.a);
  int    ang_idx = a / (2 * M_PI / ang_num);
  return node_visited_map[lround(2 * state.x)][lround(2 * state.y)][ang_idx];
}

bool PathPlanner::TiEVPlanner::is_time_out() {
#ifdef NO_TIME_LIMIT
  return (++iterations) && false;
#endif
  constexpr int mod = (1 << 10) - 1;
  if (!((++iterations) & mod)) {
    return getTimeStamp() > dead_line;
  } else
    return false;
}

double PathPlanner::TiEVPlanner::get_cost_factor(
    const astate& prev_state, const astate& now_state) const {
  const auto& weights = DecisionContext::getInstance().getPlanningWeights();
  double      result  = 1.0;
  // punish backwarding
  if (now_state.is_backward) result *= weights.w7;
  // // punish large curvature
  // result *= (1.0 + fabs(now_state.curvature) * CURVATURE_PUNISHMENT_FACTOR);
  // // punish large curvature change
  // result *= (1.0 + fabs(now_state.curvature - prev_state.curvature) *
  //                      CURVATURE_CHANGE_PUNISHMENT_FACTOR);
  // // punish history visited nodes
  // result *= (1.0 + history_visits * NODE_REVISIT_PUNISHMENT);

  return result;
}
}  // namespace TiEV