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
        while (!(is_time_out() || target_reached || node_pool.empty())) {
            // get current node
            node current = node_pool.top();
            node_pool.pop();

            // get current state
            astate current_state;
            const vector<const base_primitive*>* bases;
            // only the first node in the open list has
            // a null current.ptr
            if (current.ptr == NULL) {
                current_state = _start_state;
                bases = &base_primitives->get_nexts(current_state);
            } else {
                current_state = current.ptr->get_end_state();
                bases = &base_primitives->get_nexts(*current.ptr);
            }

            bool reverse_allowed = is_backward_enabled &&
                (current.minimum_speed == 0.0) &&
                (current.dis_after_reverse >= MIN_DISTANCE_BETWEEN_REVERSING);
            double maximum_curvature_allowed = GRID_RESOLUTION *
                max_curvature_under_velocity(current.minimum_speed);

            // for each node we caculate the probability to
            // perform analytic expansion from it.
            // the probability depends on its heuristic x.
            double x = current.score - current.cost;
            double analytic_expansion_probability = 0.2402 * exp(-0.006 * x);
            if (iterations == 1 || random_urd(gen) <=
                analytic_expansion_probability) {
                if (try_analytic_expansion(current_state,
                    reverse_allowed, maximum_curvature_allowed,
                    max_sigma_under_velocity(current.minimum_speed))) {
                    last_primitive_ptr = current.ptr;
                    if (last_primitive_ptr != NULL)
                        target_offset = last_primitive_ptr->
                            get_states().size() - 1;
                    target_reached = true;
                    break;
                }
            }

            // sin and cos values prepared for expansion
            double trans_sin, trans_cos;
            sincos(current_state.a, &trans_sin, &trans_cos);

            for (const auto* base : *bases) {
                // if this expansion changes the driving direction (backward/forward)
                bool reversed_expansion = (base->get_states().front().
                    is_backward != current_state.is_backward);
                // check if the base primitive is legal now
                if ((reverse_allowed == false && reversed_expansion) ||
                    (fabs(base->get_maximum_curvature()) > maximum_curvature_allowed))
                    continue;
                // create a new primitive from primitive pool expanded from
                // current_state, based on primitive base, and linked to
                // its parent primitive 'current.ptr'
                primitive& expansion = *(primitive_pool.make(base,
                    current.ptr, current_state, trans_sin, trans_cos));
                // check if primitive crashed
                // is_crashed does not check if the primitive is completely
                // or partly outside the local map. if the primitive is
                // not crashed but partly outside the local map, we still
                // assume that a sampled state on it might reach the target
                if (!planning_map.is_crashed(expansion)) {
                    // primitive is not crashed
                    astate end_state = expansion.get_end_state();
                    // check if primitive is near target
                    target_offset = planning_map.try_get_target_index(expansion);
                    // if target has been reached, end the searching
                    if (target_offset >= 0) {
                        last_primitive_ptr = &expansion;
                        target_reached = true;
                        break;
                    }
                    // if end_state is in the local map, add it to open list
                    else if (planning_map.is_in_map(end_state)) {
                        // update end_state to history
                        int end_state_visits = history(end_state)++;
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
                            heuristic + cost,
                            minimum_speed, cost,
                            dis_after_reverse,
                            &expansion
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

            if (last_primitive_ptr != NULL) {
                vector<astate> states(last_primitive_ptr->get_states());
                path.insert(path.end(), states.rend() - target_offset - 1, states.rend());

                last_primitive_ptr = last_primitive_ptr->get_parent();
                while (last_primitive_ptr != NULL) {
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
        double a = PathPlanner::wrap_angle_0_2_PI(state.a);
        int ang_idx = a / HISTORY_MAP_DELTA_A;
        return node_history_map[lround(state.x) >> HISTORY_MAP_SHIFT_FACTOR]
            [lround(state.y) >> HISTORY_MAP_SHIFT_FACTOR][ang_idx];
    }

    bool PathPlanner::hybrid_astar_planner::is_time_out() {
#ifdef NO_TIME_LIMIT
        return (++iterations) >= 100000;
#endif
        constexpr int mod = (1 << 10) - 1;
        if (!((++iterations) & mod)) {
            return getTimeStamp() > dead_line;
        } else return false;
    }

    bool PathPlanner::hybrid_astar_planner::try_analytic_expansion(
        const astate& state, bool backward_allowed,
        double max_curvature, double max_sigma) {

        const analytic_expansion_provider* provider = NULL;

        // since we must check whether we are approved to drive backward
        // to decide which analytic expansion provider to be used here,
        // the provider can be constructed only in the if-segment.
        // so we define an unsigned char array to pre-alloc the space
        // of candidate providers, manually call "placement new"
        // and the destructor (which is virtual) of the provider.
#if defined(USE_HC_PATH_ANALYTIC_EXPANSION)
        unsigned char _provider[
            max(sizeof(hc_reeds_shepp_path_provider),
                sizeof(cc_dubins_path_provider))];
        if (is_backward_enabled && backward_allowed)
            provider = new (_provider)
                hc_reeds_shepp_path_provider(state, target_state,
                max(max_curvature * GRID_RESOLUTION, fabs(state.curvature)),
                max_sigma * pow(GRID_RESOLUTION, 2));
        else
            provider = new (_provider)
                cc_dubins_path_provider(state, target_state,
                max(max_curvature * GRID_RESOLUTION, fabs(state.curvature)),
                max_sigma * pow(GRID_RESOLUTION, 2));
#elif defined(USE_DUBINS_ANALYTIC_EXPANSION)
        unsigned char _provider[
            max(sizeof(reeds_shepp_provider),
                sizeof(dubins_provider))];
        reeds_shepp_provider rs_provider(
            state, target_state, max_curvature * GRID_RESOLUTION);
        dubins_provider dbs_provider(
            state, target_state, max_curvature * GRID_RESOLUTION);
        if (is_backward_enabled && backward_allowed)
            provider = new (_provider) reeds_shepp_provider(
                state, target_state, max_curvature * GRID_RESOLUTION);
        else
            provider = new (_provider) dubins_provider(
                state, target_state, max_curvature * GRID_RESOLUTION);
#else
        return false;
#endif

        auto& aer = analytic_expansion_result;
        auto& map = planning_map;
        analytic_expansion_result.clear();
        if (provider->get_is_map_exceeded() == false &&
            provider->traverse(
                [&aer, &map, &state](const astate& sampled_state) {
                    aer.emplace_back(sampled_state);
                    astate& end_state = aer.back();
                    end_state.s += state.s;
                    if (map.is_in_map(end_state) == false ||
                        map.is_crashed(end_state))
                        return false;
                    else return true;
                }
            )) {
            provider->~analytic_expansion_provider();
            log_1("analytic expansion succ");
            return true;
        } else {
            provider->~analytic_expansion_provider();
        #ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
            if (analytic_expanded != NULL){
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