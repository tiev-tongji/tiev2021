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
        InitPlanner(_dynamic_obj_list,_ref_path,_start_state,
                    _start_speed_m_s,_is_backward_enabled,
                    _abs_safe_map,_lane_safe_map,
                    max_duration,_base_primitives);




    }

    void
    PathPlanner::LearnPlanner::InitPlanner(const DynamicObjList &_dynamic_obj_list, const vector<HDMapPoint> &_ref_path,
                                           const PathPlanner::astate &_start_state, double current_speed,
                                           bool _is_backward_enabled, double (*_abs_safe_map)[251],
                                           double (*_lane_safe_map)[251], time_t _max_duration,
                                           const PathPlanner::base_primitive_set *_base_primitives ) {

        constexpr double max_dcc = -0.5;
        start_time               = getTimeStamp();
        dead_line                = start_time + _max_duration;
        iterations               = 0;
        base_primitives          = _base_primitives;
        start_state              = _start_state;
        is_backward_enabled = _is_backward_enabled;
        start_speed_m_s = sqrt(max(2.0 * max_dcc * start_state.s + current_speed * current_speed,0.0));

        memset(node_visited_map, false, sizeof(node_visited_map));
        result.clear();
        analytic_expansion_result.clear();
        primitive_pool.clear();
        while(!node_pool.empty())node_pool.pop();

        planning_map.prepare(_dynamic_obj_list,_ref_path,_abs_safe_map,_lane_safe_map,_is_backward_enabled);
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


}   // namespace TiEV