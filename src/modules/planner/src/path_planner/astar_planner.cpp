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

const std::vector<PathPlanner::astate>& PathPlanner::AstarPlanner::plan(
    const astate& _start_state, const astate& _target_state,
    double current_speed, bool _is_backward_enabled,
    double (*_abs_safe_map)[MAX_COL], double (*_lane_safe_map)[MAX_COL],
    time_t _max_duration, const base_primitive_set* _base_primitives) {
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
  start_time      = getTimeStamp();
  dead_line       = start_time + _max_duration;
  iterations      = 0;
  base_primitives = _base_primitives;

  start_state         = _start_state;
  target_state        = _target_state;
  is_backward_enabled = _is_backward_enabled;

  memset(node_visited_map, false, sizeof(node_visited_map));
  result.clear();
  analytic_expansion_result.clear();
  primitive_pool.clear();

  // clear node_pool
  while (!node_pool.empty()) node_pool.pop();
  planning_map.prepare(target_state, _abs_safe_map, _lane_safe_map,
                       is_backward_enabled);
  // LOG(WARNING) << "start:" << start_state << " target:" << target_state;
#ifdef VIS_EXPANSION
  cv::circle(expansion_img, cv::Point(int(start_state.y), int(start_state.x)),
             3, cv::Scalar(0, 255, 0), -1);
  cv::circle(expansion_img, cv::Point(int(target_state.y), int(target_state.x)),
             3, cv::Scalar(0, 0, 255), -1);
#endif

  // init random floating point number generator
  std::mt19937 gen(
      start_time);  // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> random_urd;
  LOG(INFO) << "Astar Planner Initialized";

  // push start_state to node pool
  node_pool.push(
      {planning_map.get_heuristic(start_state, start_state.is_backward, 0.0),
       0.0, 0.0, 0.0, nullptr});

  bool target_reached = false;
  // last_primitive_ptr points to the last primitive
  // in the whole primitive link after target is reached
  // otherwise it is null
  primitive_ptr last_primitive_ptr = nullptr;
  // target_offset is the offset of the most close state to
  // the target in sampled_states of the last primitive.
  int target_offset = -1;
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
    bases = base_primitives->get_nexts(current_state, 0.0);
    // LOG(WARNING) << "current_state:" << current_state
    //              << " score=" << current.score;
    // usleep(100 * 1000);

    bool reverse_allowed = is_backward_enabled;

    // for each node we caculate the probability to
    // perform analytic expansion from it.
    // the probability depends on its heuristic x.
    double x                              = current.score - current.cost;
    double analytic_expansion_probability = 0.2402 * exp(-0.006 * x);
    if (iterations == 0 || random_urd(gen) <= analytic_expansion_probability) {
      if (try_analytic_expansion(
              current_state, reverse_allowed, CAR_MAX_CURVATURE,
              max_sigma() * GRID_RESOLUTION * GRID_RESOLUTION)) {
        last_primitive_ptr = current.ptr;
        if (last_primitive_ptr != NULL)
          target_offset = last_primitive_ptr->get_states().size() - 1;
        target_reached = true;
        break;
      }
    }

    for (const auto& base : bases) {
      // if this expansion changes the driving direction
      // (backward/forward)
      bool reversed_expansion =
          (base.get_states().front().is_backward != current_state.is_backward);
      // check if the base primitive is legal now
      // if ((reverse_allowed == false && reversed_expansion) ||
      //     (fabs(base.get_maximum_curvature()) > maximum_curvature_allowed))
      //   continue;
      // create a new primitive from primitive pool expanded from
      // current_state, based on primitive base, and linked to
      // its parent primitive 'current.ptr'
      primitive& expansion = *(primitive_pool.make(base, current.ptr));
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
          double heuristic =
              planning_map.get_heuristic(end_state, reverse_allowed, 0.0);
          double cost_factor = get_cost_factor(current_state, end_state);
          double cost        = current.cost + base.get_length() * cost_factor;
          double dis_after_reverse =
              base.get_length() +
              (reversed_expansion ? 0.0 : current.dis_after_reverse);
          node_pool.push(
              {heuristic + cost, 0.0, cost, dis_after_reverse, &expansion});
        }
      }
    }
  }

  log_1("end searching after ", iterations, " iterations");
  log_1("searching duration: ", (getTimeStamp() - start_time) / 1000, " ms");

  if (target_reached) {
    log_1("succ: collecting result");
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
    result.insert(result.end(), analytic_expansion_result.begin(),
                  analytic_expansion_result.end());
  } else
    log_1("timeout: no result found");
  return result;
}

void PathPlanner::AstarPlanner::visit(const astate& state) {
  double a       = PathPlanner::wrap_angle_0_2_PI(state.a);
  int    ang_idx = a / (2 * M_PI / ANGLE_NUM);
  node_visited_map[lround(2 * state.x)][lround(2 * state.y)][ang_idx] = true;
}

bool PathPlanner::AstarPlanner::is_visited(const astate& state) {
  double a       = PathPlanner::wrap_angle_0_2_PI(state.a);
  int    ang_idx = a / (2 * M_PI / ANGLE_NUM);
  return node_visited_map[lround(2 * state.x)][lround(2 * state.y)][ang_idx];
}

bool PathPlanner::AstarPlanner::is_time_out() {
#ifdef NO_TIME_LIMIT
  return (++iterations) && false;
#endif
  constexpr int mod = (1 << 10) - 1;
  if (!((++iterations) & mod)) {
    return getTimeStamp() > dead_line;
  } else
    return false;
}

bool PathPlanner::AstarPlanner::try_analytic_expansion(const astate& state,
                                                       bool   backward_allowed,
                                                       double max_curvature,
                                                       double max_sigma) {
  const analytic_expansion_provider* provider = NULL;

  // since we must check whether we are approved to drive backward
  // to decide which analytic expansion provider to be used here,
  // the provider can be constructed only in the if-segment.
  // so we define an unsigned char array to pre-alloc the space
  // of candidate providers, manually call "placement new"
  // and the destructor (which is virtual) of the provider.
#if defined(USE_HC_PATH_ANALYTIC_EXPANSION)
  unsigned char _provider[max(sizeof(hc_reeds_shepp_path_provider),
                              sizeof(cc_dubins_path_provider))];
  if (is_backward_enabled && backward_allowed)
    provider = new (_provider) hc_reeds_shepp_path_provider(
        state, target_state, max(max_curvature, fabs(state.curvature)),
        max_sigma);
  else
    provider = new (_provider) cc_dubins_path_provider(
        state, target_state, max(max_curvature, fabs(state.curvature)),
        max_sigma);
#elif defined(USE_DUBINS_ANALYTIC_EXPANSION)
  unsigned char
                       _provider[max(sizeof(reeds_shepp_provider), sizeof(dubins_provider))];
  reeds_shepp_provider rs_provider(state, target_state,
                                   max_curvature * GRID_RESOLUTION);
  dubins_provider      dbs_provider(state, target_state,
                               max_curvature * GRID_RESOLUTION);
  if (is_backward_enabled && backward_allowed)
    provider = new (_provider) reeds_shepp_provider(
        state, target_state, max_curvature * GRID_RESOLUTION);
  else
    provider = new (_provider)
        dubins_provider(state, target_state, max_curvature * GRID_RESOLUTION);
#else
  return false;
#endif

  auto& aer = analytic_expansion_result;
  auto& map = planning_map;
  analytic_expansion_result.clear();
  if (provider->get_is_map_exceeded() == false &&
      provider->traverse([&aer, &map, &state](const astate& sampled_state) {
        aer.emplace_back(sampled_state);
        astate& end_state = aer.back();
        end_state.s += state.s;
        if (map.is_in_map(end_state) == false || map.is_lane_crashed(end_state))
          return false;
        else
          return true;
      })) {
    provider->~analytic_expansion_provider();
    log_1("analytic expansion succ");
    return true;
  } else {
    provider->~analytic_expansion_provider();
#ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
    if (analytic_expanded != NULL) {
      vector<pair<double, double>> xys;
      xys.reserve(analytic_expansion_result.size());
      for (const auto& p : analytic_expansion_result)
        xys.emplace_back(p.x, p.y);
      analytic_expanded(xys);
    }
#endif
    analytic_expansion_result.clear();
    return false;
  }
}

double PathPlanner::AstarPlanner::get_cost_factor(
    const astate& prev_state, const astate& now_state) const {
  double result = 1.0;
  // punish backwarding
  if (now_state.is_backward) result *= BACKWARD_COST_FACTOR;
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