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
        double _start_speed_m_s,
        bool _is_backward_enabled,
        double (*_safe_map)[MAX_COL],
        time_t max_duration) {

        have_result = false;
        start_time = getTimeStamp();
        dead_line = start_time + max_duration;
        iterations = 0;

        start_state = _start_state;
        target_state = _target_state;
        start_speed_m_s = _start_speed_m_s;
        _is_backward_enabled |= start_state.is_backward;
        is_backward_enabled = _is_backward_enabled;

        memset(node_history_map, 0, sizeof(node_history_map));
        result.clear();
        analytic_expansion_result.clear();
        primitive_pool.clear();

        // clear node_pool
        vector<node> node_pool_storage;
        while (!node_pool.empty()) node_pool.pop();
        planning_map.init(target_state, _safe_map,
            is_backward_enabled, BACKWARD_COST_FACTOR);

        log(1, "hybrid astar planner initialized");

        // push start_state to node pool
        node_pool.push({
            planning_map.get_heuristic(start_state, start_state.is_backward),
            start_speed_m_s, 0.0, 0.0,
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
                ++history(current_state);
            }

            if (try_analytic_expansion(current_state, current)) {
                last_primitive_ptr = current.ptr;
                if (!last_primitive_ptr.is_null())
                    target_offset = last_primitive_ptr->
                        get_states().size() - 1;
                target_reached = true;
                break;
            }

            // get base primitives to expand current state
            const vector<const base_primitive*>* bases;
            if (current.ptr.is_null())
                bases = &base_primitives->get_nexts(current_state);
            else
                bases = &base_primitives->get_nexts(*current.ptr);

            bool reverse_allowed = is_backward_enabled &&
                (current.minimum_speed == 0.0) &&
                (current.dis_after_reverse >= MIN_DISTANCE_BETWEEN_REVERSING);
            double maximum_curvature_allowed = GRID_RESOLUTION * TiEV::GRAVITY *
                TiEV::MIU / (current.minimum_speed * current.minimum_speed);

            for (const auto* base : *bases) {
                // if this expansion changes the driving direction (backward/forward)
                bool reversed_expansion = (base->get_states().front().
                    is_backward != current_state.is_backward);
                // check if the base primitive is legal now
                if ((reverse_allowed == false && reversed_expansion) ||
                    (fabs(base->get_maximum_curvature()) > maximum_curvature_allowed))
                    continue;

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
                    // if target has been reached
                    if (target_offset >= 0) {
                        last_primitive_ptr = primitive_ptr(&primitive_pool,
                            primitive_pool.size() - 1);
                        target_reached = true;
                        break;
                    }
                    // else create node and push it to queue
                    else {
                        double heuristic = planning_map.get_heuristic(end_state, reverse_allowed);
                        double minimum_speed = sqrt(max(0.0,
                            (current.minimum_speed * current.minimum_speed - 2 *
                            base->get_length() * (GRID_RESOLUTION * SPEED_DESCENT_FACTOR))
                        )); // v_t = v_0 + at = \sqrt{{v_0}^2 + 2as}
                        double cost_factor =
                            (end_state.is_backward ? BACKWARD_COST_FACTOR : 1.0) *
                            (1.0 + CURVATURE_PUNISHMENT_FACTOR * max(0.0, base->
                                get_average_curvature() - MIN_CURVATURE_TO_PUNISH));
                        double cost = current.cost + base->get_length() * cost_factor;
                        double dis_after_reverse = base->get_length() +
                            reversed_expansion ? 0.0 : current.dis_after_reverse;
                        node_pool.push({
                            heuristic + cost, minimum_speed, cost,
                            dis_after_reverse,
                            primitive_ptr(&primitive_pool, primitive_pool.size() - 1)
                        });
                    }
                }
            }
        }

        log(1, "end searching after ", iterations, " iterations");
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
                for (int k = 0; k < HISTORY_MAP_DEPTH; ++k)
                    output_map[i][j] += node_history_map
                        [i >> HISTORY_MAP_SHIFT_FACTOR]
                        [j >> HISTORY_MAP_SHIFT_FACTOR][k];
    }

    void PathPlanner::hybrid_astar_planner::merge_xya_distance_map(
        pair<double, double> (*output_map)[MAX_COL]) const {
        planning_map.merge_xya_distance_map(output_map);
    }

    int& PathPlanner::hybrid_astar_planner::history(astate& state) {
        if (!(state.a >= 0 && state.a < (2 * M_PI)))
            state.a -= floor(state.a / (2 * PI)) * (2 * PI);
        int ang_idx = state.a / HISTORY_MAP_DELTA_A;
        return node_history_map[lround(state.x) >> HISTORY_MAP_SHIFT_FACTOR]
            [lround(state.y) >> HISTORY_MAP_SHIFT_FACTOR][ang_idx];
    }

    bool PathPlanner::hybrid_astar_planner::is_time_out() {
#ifdef NO_TIME_LIMIT
        return (++iterations) >= 1000000;
#endif
        constexpr int mod = (1 << 11) - 1;
        if (!((++iterations) & mod)) {
            return getTimeStamp() > dead_line;
        } else return false;
    }

    bool PathPlanner::hybrid_astar_planner::
        try_analytic_expansion(
            const astate& state,
            const node& node) {
#ifdef NO_ANALYTIC_EXPANSION
        return false;
#endif
        // https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
        static random_device rd;
        static mt19937 gen(rd());
        static uniform_real_distribution<> random_float;
        double heuristic = node.score - node.cost;

        if (random_float(gen) > 1.0 / heuristic)
            return false;

        double curvature = 1.0 / (5.0 / GRID_RESOLUTION);
        curvature = min(curvature, 1.0 / (5.0 / GRID_RESOLUTION));

        analytic_expansion_result.clear();
        analytic_expansion_provider* provider;

        if (is_backward_enabled) {
            rs.prepare(state, target_state, curvature);
            provider = &rs;
        } else {
            dubins.prepare(state, target_state, curvature);
            provider = &dubins;
        }

        double safe_distance = state.s - 1.0;
        while (provider->get_next_state(analytic_expansion_result.emplace_back())) {
            const astate& back_state = analytic_expansion_result.back();

            if (planning_map.is_in_map(back_state)) {
                if (back_state.s <= safe_distance) continue;
                else if (!planning_map.is_crashed(back_state)) {
                    safe_distance = back_state.s + planning_map.
                        get_maximum_safe_distance(back_state);
                    continue;
                }
            }

            analytic_expansion_result.clear();
            return false;
        }

        analytic_expansion_result.pop_back();
        return true;
    }
}