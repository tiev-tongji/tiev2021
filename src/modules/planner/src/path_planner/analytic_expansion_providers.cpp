#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"

namespace TiEV {
    PathPlanner::dubins_provider::dubins_provider(
        const astate& _start_state, const astate& _end_state, double _max_curvature)
        : dubins_space(_max_curvature, ANALYTIC_EXPANSION_SAMPLING_STEP) {
        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };
        start_cost = _start_state.s;

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = dubins_space.get_controls(start_state, end_state);

        target_length = 0.0;
        for (const auto& control : controls)
            target_length += fabs(control.delta_s);
        current_s = -ANALYTIC_EXPANSION_SAMPLING_STEP;
    }

    bool PathPlanner::dubins_provider::get_next_state(astate& output_state) {
        current_s = min(target_length, current_s +
            ANALYTIC_EXPANSION_SAMPLING_STEP);

        double rel_s = current_s / target_length;
        State state = dubins_space.interpolate(start_state, controls, rel_s);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;

        output_state.s = start_cost + current_s;
        return current_s != target_length;
    }

    PathPlanner::reeds_shepp_provider::reeds_shepp_provider(
        const astate& _start_state, const astate& _end_state, double _max_curvature) :
        rs_space(_max_curvature, ANALYTIC_EXPANSION_SAMPLING_STEP) {
        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };
        start_cost = _start_state.s;

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = rs_space.get_controls(start_state, end_state);

        target_length = 0.0;
        for (const auto& control : controls)
            target_length += fabs(control.delta_s);
        current_s = -ANALYTIC_EXPANSION_SAMPLING_STEP;
    }

    bool PathPlanner::reeds_shepp_provider::get_next_state(astate& output_state) {
        current_s = min(target_length, current_s +
            ANALYTIC_EXPANSION_SAMPLING_STEP);

        double rel_s = current_s / target_length;
        State state = rs_space.interpolate(start_state, controls, rel_s);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;

        output_state.s = start_cost + current_s;
        return current_s != target_length;
    }

    PathPlanner::cc_dubins_path_provider::cc_dubins_path_provider(
        const astate& _start_state, const astate& _end_state,
        double _max_curvature, double _max_sigma) :
        cc_dubins_space(_max_curvature, _max_sigma,
            ANALYTIC_EXPANSION_SAMPLING_STEP) {

        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };
        start_cost = _start_state.s;

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = cc_dubins_space.get_controls(start_state, end_state);

        target_length = 0.0;
        for (const auto& control : controls)
            target_length += fabs(control.delta_s);
        current_s = -ANALYTIC_EXPANSION_SAMPLING_STEP;
    }

    bool PathPlanner::cc_dubins_path_provider::get_next_state(astate& output_state) {
        current_s = min(target_length, current_s +
            ANALYTIC_EXPANSION_SAMPLING_STEP);

        double rel_s = current_s / target_length;
        State state = cc_dubins_space.interpolate(start_state, controls, rel_s);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;

        output_state.s = start_cost + current_s;
        return current_s != target_length;
    }

    PathPlanner::hc_reeds_shepp_path_provider::hc_reeds_shepp_path_provider(
        const astate& _start_state, const astate& _end_state,
        double _max_curvature, double _max_sigma) :
        hc_rs_space(_max_curvature, _max_sigma,
            ANALYTIC_EXPANSION_SAMPLING_STEP) {
        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };
        start_cost = _start_state.s;

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = hc_rs_space.get_controls(start_state, end_state);

        target_length = 0.0;
        for (const auto& control : controls)
            target_length += fabs(control.delta_s);
        current_s = -ANALYTIC_EXPANSION_SAMPLING_STEP;
    }

    bool PathPlanner::hc_reeds_shepp_path_provider::get_next_state(astate& output_state) {
        current_s = min(target_length, current_s +
            ANALYTIC_EXPANSION_SAMPLING_STEP);

        double rel_s = current_s / target_length;
        State state = hc_rs_space.interpolate(start_state, controls, rel_s);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;

        output_state.s = start_cost + current_s;
        return current_s != target_length;
    }
}