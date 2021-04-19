#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "math.h"
#include "log.h"
#include <random>

namespace TiEV {

#ifdef DEBUG_EXPANSION_CALLBACK
    PathPlanner::node_expansion_callback node_expanded;
#endif

#ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
    PathPlanner::analytic_expansion_callback analytic_expanded;
#endif

    void PathPlanner::hybrid_astar_planner::plan(
        const astate& _start_state,
        const astate& _target_state,
        double _start_speed_m_s,
        bool _is_backward_enabled,
        double (*_safe_map)[MAX_COL],
        time_t _max_duration,
        const base_primitive_set* _base_primitives) {

        have_result = false;
        start_time = getTimeStamp();
        dead_line = start_time + _max_duration;
        iterations = 0;
        base_primitives = _base_primitives;

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
        while (!node_pool.empty()) node_pool.pop();
        planning_map.prepare(target_state, _safe_map,
            is_backward_enabled);

        // init random floating point number generator
        std::mt19937 gen(start_time); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> random_urd;

        log_1("hybrid astar planner initialized");

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
            double maximum_curvature_allowed = GRID_RESOLUTION *
                max_curvature_under_velocity(current.minimum_speed);

            double x = current.score - current.cost;
            double analytic_expansion_probability = 0.2402 * exp(-0.006 * x);
            if (random_urd(gen) <= analytic_expansion_probability &&
                try_analytic_expansion(current_state,
                reverse_allowed, maximum_curvature_allowed,
                max_sigma_under_velocity(current.minimum_speed))) {
                last_primitive_ptr = current.ptr;
                if (!last_primitive_ptr.is_null())
                    target_offset = last_primitive_ptr->
                        get_states().size() - 1;
                target_reached = true;
                break;
            }

            for (const auto* base : *bases) {
                // if this expansion changes the driving direction (backward/forward)
                bool reversed_expansion = (base->get_states().front().
                    is_backward != current_state.is_backward);
                // check if the base primitive is legal now
                if ((reverse_allowed == false && reversed_expansion) ||
                    (fabs(base->get_maximum_curvature()) > maximum_curvature_allowed))
                    continue;

                if (reversed_expansion)
                    reversed_expansion = true;

                // create primitive from base
                primitive expansion(base, current.ptr, current_state);
                // check if primitive crashed
                if (!planning_map.is_crashed(expansion)) {
                    // primitive is not crashed
                    astate end_state = expansion.get_end_state();
                    // update end_state to history
                    int end_state_visits = history(end_state)++;
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
                        double cost_factor = get_cost_factor(current_state, end_state, end_state_visits);
                        double cost = current.cost + base->get_length() * cost_factor;
                        double dis_after_reverse = base->get_length() +
                            (reversed_expansion ? 0.0 : current.dis_after_reverse);
                    #ifdef DEBUG_EXPANSION_CALLBACK
                        if (node_expanded != NULL)
                            node_expanded(end_state.x, end_state.y, end_state.a,
                                end_state.curvature, end_state.is_backward,
                                heuristic, cost);
                    #endif
                        node_pool.push({
                            heuristic * 1.1 + cost,
                            minimum_speed, cost,
                            dis_after_reverse,
                            primitive_ptr(&primitive_pool, primitive_pool.size() - 1)
                        });
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
        } else log_1("timeout: no result found");
    }

    bool PathPlanner::hybrid_astar_planner::get_have_result() const {
        return have_result;
    }

    const vector<PathPlanner::astate>& PathPlanner::hybrid_astar_planner::get_result() const {
        if (have_result)
            return result;
        else {
            log_0("calling get_result() throws an exception when get_have_result() returns false");
            throw exception();
        };
    }

    void PathPlanner::hybrid_astar_planner::merge_xya_distance_map(
        pair<double, double> (*output_map)[MAX_COL]) const {
        planning_map.merge_xya_distance_map(output_map);
    }

    int& PathPlanner::hybrid_astar_planner::history(const astate& state) {
        double a = state.a - floor(state.a / (2 * PI)) * (2 * PI);
        int ang_idx = a / HISTORY_MAP_DELTA_A;
        return node_history_map[lround(state.x) >> HISTORY_MAP_SHIFT_FACTOR]
            [lround(state.y) >> HISTORY_MAP_SHIFT_FACTOR][ang_idx];
    }

    bool PathPlanner::hybrid_astar_planner::is_time_out() {
#ifdef NO_TIME_LIMIT
        return (++iterations) >= 100000;
#endif
        constexpr int mod = (1 << 11) - 1;
        if (!((++iterations) & mod)) {
            return getTimeStamp() > dead_line;
        } else return false;
    }

    bool PathPlanner::hybrid_astar_planner::try_analytic_expansion(
        const astate& state, bool backward_allowed,
        double max_curvature, double max_sigma) {
        unique_ptr<analytic_expansion_provider> provider;

#if defined(USE_HC_PATH_ANALYTIC_EXPANSION)
        if (is_backward_enabled && backward_allowed)
            provider = std::move(make_unique<hc_reeds_shepp_path_provider>(
                state, target_state,
                max(max_curvature * GRID_RESOLUTION, fabs(state.curvature)),
                max_sigma * pow(GRID_RESOLUTION, 2)
            ));
        else
            provider = std::move(make_unique<cc_dubins_path_provider>(
                state, target_state,
                max(max_curvature * GRID_RESOLUTION, fabs(state.curvature)),
                max_sigma * pow(GRID_RESOLUTION, 2)
            ));
#elif defined(USE_DUBINS_ANALYTIC_EXPANSION)
        if (is_backward_enabled && backward_allowed)
            provider = std::move(make_unique<reeds_shepp_provider>(
                state, target_state, max_curvature * GRID_RESOLUTION));
        else
            provider = std::move(make_unique<dubins_provider>(
                state, target_state, max_curvature * GRID_RESOLUTION));
#else
        return false;
#endif

        analytic_expansion_result.clear();
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

        #ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
            if (analytic_expanded != NULL) {
                vector<pair<double, double>> xys;
                xys.reserve(analytic_expansion_result.size());
                for (const auto& state : analytic_expansion_result)
                    xys.emplace_back(state.x, state.y);
                analytic_expanded(xys);
            }
        #endif
            analytic_expansion_result.clear();
            return false;
        }

        analytic_expansion_result.pop_back();
        log_1("analytic expansion succ");
        return true;
    }

    double PathPlanner::hybrid_astar_planner::get_cost_factor(
        const astate& prev_state, const astate& now_state, int history_visits) const {
        double result = 1.0;
        // punish backwarding
        if (now_state.is_backward)
            result *= BACKWARD_COST_FACTOR;
        // punish large curvature
        result *= (1.0 + fabs(now_state.curvature) * CURVATURE_PUNISHMENT_FACTOR);
        // punish large curvature change
        result *= (1.0 + fabs(now_state.curvature - prev_state.curvature) *
            CURVATURE_CHANGE_PUNISHMENT_FACTOR);
        // punish history visited nodes
        result *= (1.0 + history_visits * NODE_REVISIT_PUNISHMENT);

        return result;
    }
}