#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "math.h"
#include "log.h"
#include <random>

namespace TiEV {
    PathPlanner::hybrid_astar_planner::hybrid_astar_planner(
        const base_primitive_set* _base_primitives
    ) : base_primitives(_base_primitives), have_result(false) { }

    void PathPlanner::hybrid_astar_planner::plan(
        const astate& _start_state,
        const astate& _target_state,
        double (*_safe_map)[MAX_COL],
        time_t max_duration) {

        have_result = false;
        start_time = getTimeStamp();
        dead_line = start_time + max_duration;
        iterations = 0;

        start_state = _start_state;
        target_state = _target_state;
        memset(node_history_map, 0, sizeof(node_history_map));
        result.clear();
        analytic_expansion_result.clear();
        primitive_pool.clear();
        while (!node_pool.empty()) node_pool.pop();
        planning_map.init(target_state, _safe_map);

        log(1, "hybrid astar planner initialized");

        // push start_state to node pool
        node_pool.push({
            planning_map.get_heuristic(start_state),
            primitive_ptr()
        });

        bool target_reached = false;
        // last_primitive_ptr points to the last primitive
        // in the whole primitive link after target is reached
        // otherwise it is null
        primitive_ptr last_primitive_ptr;
        // target_offset is the offset of the most close state to
        // the target in sampled_states of the last primitive.
        int target_offset = -1;
        while (!(is_time_out() || target_reached)) {

            // get current node
            node current = node_pool.top();
            node_pool.pop();

            // get current state
            astate current_state;
            if (current.ptr.is_null()) {
                current_state = _start_state;
            } else {
                current_state = current.ptr->get_end_state();
                record_history(current_state);
            }

            if (try_analytic_expansion(current_state,
                current.score - current_state.s)) {
                last_primitive_ptr = current.ptr;
                if (!last_primitive_ptr.is_null())
                    target_offset = last_primitive_ptr->
                        get_states().size() - 1;
                target_reached = true;
                break;
            }

            // expand current state
            const vector<const base_primitive*>* bases;
            if (current.ptr.is_null())
                bases = &base_primitives->get_nexts(current_state);
            else bases = &base_primitives->get_nexts(*current.ptr);

            for (const auto* base : *bases) {
                // create primitive from base
                primitive expansion(base, current.ptr, current_state);
                // check if primitive crashed
                if (!planning_map.is_crashed(expansion)) {
                    // primitive is not crashed
                    astate end_state = expansion.get_end_state();
                    // check if primitive is near target
                    target_offset = planning_map.try_get_target_index(expansion);
                    // add primitive into pool
                    primitive_pool.emplace_back(move(expansion));

                    if (target_offset >= 0) {
                        last_primitive_ptr = primitive_ptr(&primitive_pool,
                            primitive_pool.size() - 1);
                        target_reached = true;
                        break;
                    }
                    // else create node and push it to queue
                    else node_pool.push({
                        planning_map.get_heuristic(end_state) + end_state.s + node_history_map
                                [(int)round(end_state.x) >> HISTORY_MAP_SHIFT_FACTOR]
                                [(int)round(end_state.y) >> HISTORY_MAP_SHIFT_FACTOR] * 5,
                        primitive_ptr(&primitive_pool, primitive_pool.size() - 1)
                    });
                }
            }
        }

        log(1, "end searching after ", iterations, "iterations");
        log(1, "searching duration: ", (getTimeStamp() - start_time) / 1000, " ms");

        if (target_reached) {
            log(1, "succ: collecting result");
            // path stores the reversed states list
            vector<astate> path;

            if (!last_primitive_ptr.is_null()) {
                vector<astate> states(last_primitive_ptr->get_states());
                path.insert(path.end(), states.rend() - target_offset - 1, states.rend());

                last_primitive_ptr = last_primitive_ptr->get_parent();
                while (!last_primitive_ptr.is_null()) {
                    vector<astate> states = last_primitive_ptr->get_states();
                    path.insert(path.end(), states.rbegin() + 1, states.rend());
                    last_primitive_ptr = last_primitive_ptr->get_parent();
                }
            }

            result.insert(result.begin(), path.rbegin(), path.rend());
            result.insert(result.end(), analytic_expansion_result.begin(),
                analytic_expansion_result.end());
            have_result = true;
        } else log(1, "timeout: no result found");
    }

    bool PathPlanner::hybrid_astar_planner::get_have_result() const {
        return have_result;
    }

    const vector<PathPlanner::astate>& PathPlanner::hybrid_astar_planner::get_result() const {
        if (have_result)
            return result;
        else {
            log(0, "calling get_result() throws an exception when get_have_result() returns false");
            throw exception();
        };
    }

    void PathPlanner::hybrid_astar_planner::merge_history_map(int (*output_map)[MAX_COL]) const {
        for (int i = 0; i < MAX_ROW; ++i)
            for (int j = 0; j < MAX_COL; ++j)
                output_map[i][j] += node_history_map
                    [i >> HISTORY_MAP_SHIFT_FACTOR]
                    [j >> HISTORY_MAP_SHIFT_FACTOR];
    }

    void PathPlanner::hybrid_astar_planner::merge_xy_distance_map(
        double (*output_map)[MAX_COL]) const {
        planning_map.merge_xy_distance_map(output_map);
    }

    void PathPlanner::hybrid_astar_planner::merge_xya_distance_map(
        pair<double, double> (*output_map)[MAX_COL]) const {
        planning_map.merge_xya_distance_map(output_map);
    }

    void PathPlanner::hybrid_astar_planner::record_history(const astate& state) {
        const int hist_x = (int)round(state.x) >> HISTORY_MAP_SHIFT_FACTOR;
        const int hist_y = (int)round(state.y) >> HISTORY_MAP_SHIFT_FACTOR;
        ++ node_history_map[hist_x][hist_y];
    }

    bool PathPlanner::hybrid_astar_planner::is_time_out() {
#ifdef NO_TIME_LIMIT
        return false;
#endif
        constexpr int mod = (1 << 11) - 1;
        if (!((++iterations) & mod)) {
            if (iterations > 2000000) return true;
            return getTimeStamp() > dead_line;
        } else return false;
    }

    bool PathPlanner::hybrid_astar_planner::
        try_analytic_expansion(const astate& state, double heuristic) {
#ifdef NO_ANALYTIC_EXPANSION
        return false;
#endif
        // https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
        static random_device rd;
        static mt19937 gen(rd());
        static uniform_real_distribution<> random_float;

        if (random_float(gen) > 1.0 / heuristic)
            return false;

        double       q0[] = { state.x, state.y, state.a };
        double       q1[] = { target_state.x, target_state.y, target_state.a };
        const double step = base_primitives->get_sampling_step_size();
        double       q[3], last_q[3] = { state.x, state.y, state.a }, x = step, length;
        DubinsPath tmpath;
        dubins_shortest_path(&tmpath, q0, q1, 25);
        length = dubins_path_length(&tmpath);
        astate tmp;
        while(x < length) {
            dubins_path_sample(&tmpath, x, q);
            tmp.x = q[0]; tmp.y = q[1]; tmp.a = q[2];
            if(planning_map.is_in_map(tmp) && !planning_map.is_crashed(tmp)) {
                tmp.curvature = ((q[2] == last_q[2]) ? 0.0 :
                    (q[2] > last_q[2] ? 0.04 : -0.04));
                tmp.s = state.s + x;
                tmp.is_backward  = (((q[0] - last_q[0]) * cos(q[2]) + (q[1] - last_q[1]) * sin(q[2])) < 0);
                analytic_expansion_result.push_back(tmp);
                memcpy(last_q, q, 3 * sizeof(double));
            } else {
                analytic_expansion_result.clear();
                return false;
            }
            x += step;
        }
        return true;
    }
}