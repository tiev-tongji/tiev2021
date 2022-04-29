#include <random>
#include <cmath>

#include "log.h"
#include "opencv2/opencv.hpp"
#include "path_planner.h"
#include "tievlog.h"

namespace TiEV{

    const vector<PathPlanner::astate> &
    PathPlanner::LearnPlanner::plan(const DynamicObjList &_dynamic_obj_list, const vector<HDMapPoint> &_ref_path,
                                    const PathPlanner::astate &_start_state, double _start_speed_m_s,
                                    bool _is_backward_enabled, double (*_abs_safe_map)[251], double (*_lane_safe_map)[251],
                                    time_t max_duration, const PathPlanner::base_primitive_set *_base_primitives,
                                    bool *plan_in_time) {
        LOG(INFO) << "call plan " << endl;

        InitPlanner(_dynamic_obj_list,_ref_path,_start_state,
                    _start_speed_m_s,_is_backward_enabled,
                    _abs_safe_map,_lane_safe_map,
                    max_duration,_base_primitives);
        (*plan_in_time) = SovleRusultPath();
        return result;
    }
    bool
    PathPlanner::LearnPlanner::SovleRusultPath(){
        node_pool.push({planning_map.get_heuristic(
                start_state, start_state.is_backward, start_speed_m_s),
                        start_speed_m_s, 0.0, 0.0, nullptr});
        bool target_reached = false;
        while (!is_time_out() && !target_reached && !node_pool.empty()) {
            node current = node_pool.top();
            node_pool.pop();
            target_reached = extendNodeIntoPool(current);
        }


        vector<astate> path;
        if (last_primitive_ptr != nullptr) {
            vector<astate> states(last_primitive_ptr->get_states());
            int target_offset = planning_map.try_get_target_index(*last_primitive_ptr);
            path.insert(path.end(), states.rend() - target_offset - 1, states.rend());
            last_primitive_ptr = last_primitive_ptr->get_parent();
            while (last_primitive_ptr != nullptr) {
                vector<astate> states_tmp = last_primitive_ptr->get_states();
                path.insert(path.end(), states_tmp.rbegin() + 1, states_tmp.rend());
                last_primitive_ptr = last_primitive_ptr->get_parent();
            }
        }
        result.insert(result.begin(),path.rbegin(),path.rend());
        LOG(INFO)<<" target is "<< target_reached <<endl;
        return target_reached;
    }


    void
    PathPlanner::LearnPlanner::InitPlanner(const DynamicObjList &_dynamic_obj_list, const vector<HDMapPoint> &_ref_path,
                                           const PathPlanner::astate &_start_state, double _current_speed,
                                           bool _is_backward_enabled, double (*_abs_safe_map)[251],
                                           double (*_lane_safe_map)[251], time_t _max_duration,
                                           const PathPlanner::base_primitive_set *_base_primitives ) {

        start_time               = getTimeStamp();
        dead_line                = start_time + _max_duration;
        iterations               = 0;
        base_primitives          = _base_primitives;
        start_state              = _start_state;
        ref_path = _ref_path;
        current_speed = _current_speed;
        is_backward_enabled = _is_backward_enabled;
        last_primitive_ptr = nullptr;

        start_speed_m_s = sqrt(max(2.0 * max_dcc * start_state.s + current_speed * current_speed,0.0));

        memset(node_visited_map, false, sizeof(node_visited_map));
        result.clear();
        analytic_expansion_result.clear();
        primitive_pool.clear();
        while(!node_pool.empty())node_pool.pop();

        planning_map.prepare(_dynamic_obj_list,_ref_path,
                             _abs_safe_map,_lane_safe_map,_is_backward_enabled);
    }

    double PathPlanner::LearnPlanner::getNearestStateCosFromRef(const PathPlanner::astate &state, const vector<HDMapPoint>& ref_path) {
        double min_dis = 1e8;
        double cos_val = -1;
        for (const auto& ref_p : ref_path) {
            const double sqr_dis = sqrDistance(ref_p.x, ref_p.y, state.x, state.y);
            if (sqr_dis < min_dis) {
                min_dis = sqr_dis;
                cos_val = cos(state.ang - ref_p.ang);
            }
        }
        return cos_val;
    }

    bool PathPlanner::LearnPlanner::extendNodeIntoPool(node& current) {
        astate current_state = (current.ptr == nullptr ? start_state: current.ptr->get_end_state());
        const double state_possible_speed = sqrt(max(
                2.0 * max_dcc * current_state.s + current_speed * current_speed,
                0.0));  // v = sqrt(2*a*s+ v_0^2)
        double max_cos_with_ref_path = -1;
        for (const base_primitive& base : base_primitives->get_nexts(current_state, state_possible_speed)) {
            bool reversed_expansion = (base.get_states().front().is_backward != current_state.is_backward);
            primitive& expansion = *(primitive_pool.make(base, current.ptr));
            if (current_speed < 3) {
                const double cos_with_ref =
                        getNearestStateCosFromRef(expansion.get_end_state(),ref_path);
                if (cos_with_ref > max_cos_with_ref_path) {
                    last_primitive_ptr = &expansion;
                    max_cos_with_ref_path = cos_with_ref;
                }
            }
            if (isVisited(expansion.get_end_state())) continue;
            // check if primitive crashed
            // is_crashed does not check if the primitive is completely
            // or partly outside the local map. if the primitive is
            // not crashed but partly outside the local map, we still
            // assume that a sampled state on it might reach the target
            if (!planning_map.is_lane_crashed(expansion)) {
                astate end_state = expansion.get_end_state();
                // target_offset is the offset of the most close state to
                // the target in sampled_states of the last primitive.
                int target_offset = planning_map.try_get_target_index(expansion);
                if (target_offset >= 0) {
                    last_primitive_ptr = &expansion;
                    return true;
                }
                else if (planning_map.is_in_map(end_state)) {
                    // update end_state to history
                    setVisit(end_state);
                    double heuristic = planning_map.get_heuristic(
                            end_state, is_backward_enabled, current_speed);
                    double cost_factor = get_cost_factor(current_state, end_state);
                    double cost        = current.cost + base.get_length() * cost_factor;
                    double dis_after_reverse =base.get_length() + (reversed_expansion ? 0.0 : current.dis_after_reverse);
                    node_pool.push({heuristic + cost, state_possible_speed, cost,
                                    dis_after_reverse, &expansion});
                }
            }
        }
        return false;
    }

    void PathPlanner::LearnPlanner::setVisit(const PathPlanner::astate &state) {

        double a       = PathPlanner::wrap_angle_0_2_PI(state.ang);
        int    ang_idx = static_cast<int>(a / (2 * M_PI / ang_num));
        node_visited_map[lround(2 * state.x)][lround(2 * state.y)][ang_idx] = true;
    }

    bool PathPlanner::LearnPlanner::isVisited(const astate& state) {
        double a       = PathPlanner::wrap_angle_0_2_PI(state.ang);
        int    ang_idx = static_cast<int>( a / (2 * M_PI / ang_num)) ;
        return node_visited_map[lround(2 * state.x)][lround(2 * state.y)][ang_idx];
    }


    bool PathPlanner::LearnPlanner::is_time_out() {
        constexpr int mod = (1 << 10) - 1;
        if (!((++iterations) & mod)) {
            return getTimeStamp() > dead_line;
        } else
            return false;
    }


    double PathPlanner::LearnPlanner::get_cost_factor(
            const astate& prev_state, const astate& now_state) {
        const auto& weights = DecisionContext::getInstance().getPlanningWeights();
        double      cost_fac  = 1.0;
        // punish back warding
        if (now_state.is_backward) cost_fac *= weights.w7;
        return cost_fac;
    }

}   // namespace TiEV