#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"

namespace TiEV {

    double _get_length(const std::vector<steer::Control>& controls) {
        double l = 0.0;
        for (const auto& control : controls)
            l += fabs(control.delta_s);
        return l;
    }

    template<typename space_type> bool _get_is_map_exceeded(
        const space_type& space, const steer::State& start_state,
        const vector<steer::Control>& controls) {
        steer::State current_state(start_state);
        for (const auto& control : controls) {
            current_state = space.integrate_ODE(
                current_state, control, fabs(control.delta_s));
            int ridx = (int) round(current_state.x);
            int cidx = (int) round(current_state.y);
            if (ridx < 0 || ridx >= MAX_ROW || cidx < 0 || cidx >= MAX_COL)
                return true;
        }
        return false;
    }

    template<typename space_type, typename state_type> bool _traverse(
        const space_type& space,
        const steer::State& start_state,
        const vector<steer::Control>& controls,
        const function<bool (const state_type&)>& callback,
        const double step) {
        double s = 0.0;
        State last_state = start_state;
        for (const auto& control : controls) {
            double clength = fabs(control.delta_s);
            double offset = 0.0;
            while (offset < clength) {
                double _step = min(clength - offset, step);
                offset += _step;
                s += _step;
                last_state = space.integrate_ODE(
                    last_state, control, _step);
                if (!callback({
                    last_state.x, last_state.y, last_state.theta,
                    s, last_state.kappa, last_state.d < 0.0 })
                ) return false;
            }
        }
        return true;
    }

    PathPlanner::dubins_provider::dubins_provider(
        const astate& _start_state, const astate& _end_state, double _max_curvature)
        : dubins_space(_max_curvature, ANALYTIC_EXPANSION_SAMPLING_STEP) {
        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = dubins_space.get_controls(start_state, end_state);
    }

    double PathPlanner::dubins_provider::get_length() const {
        return _get_length(controls); }

    bool PathPlanner::dubins_provider::get_is_map_exceeded() const {
        return _get_is_map_exceeded(dubins_space, start_state, controls); }

    bool PathPlanner::dubins_provider::traverse(
        const function<bool (const astate&)>& callback) const {
        return _traverse(dubins_space, start_state, controls,
            callback, ANALYTIC_EXPANSION_SAMPLING_STEP);
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

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = rs_space.get_controls(start_state, end_state);
    }

    double PathPlanner::reeds_shepp_provider::get_length() const {
        return _get_length(controls); }

    bool PathPlanner::reeds_shepp_provider::get_is_map_exceeded() const {
        return _get_is_map_exceeded(rs_space, start_state, controls); }

    bool PathPlanner::reeds_shepp_provider::traverse(
        const function<bool (const astate&)>& callback) const {
        return _traverse(rs_space, start_state, controls,
            callback, ANALYTIC_EXPANSION_SAMPLING_STEP);
    }

    PathPlanner::cc_dubins_path_provider::cc_dubins_path_provider(
        const astate& _start_state, const astate& _end_state,
        double _max_curvature, double _max_sigma) :
        cc_dubins_space(_max_curvature, _max_sigma) {

        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = cc_dubins_space.get_controls(start_state, end_state);
    }

    double PathPlanner::cc_dubins_path_provider::get_length() const {
        return _get_length(controls); }

    bool PathPlanner::cc_dubins_path_provider::get_is_map_exceeded() const {
        return _get_is_map_exceeded(cc_dubins_space, start_state, controls); }

    bool PathPlanner::cc_dubins_path_provider::traverse(
        const function<bool (const astate&)>& callback) const {
        return _traverse(cc_dubins_space, start_state, controls,
            callback, ANALYTIC_EXPANSION_SAMPLING_STEP);
    }

    PathPlanner::hc_reeds_shepp_path_provider::hc_reeds_shepp_path_provider(
        const astate& _start_state, const astate& _end_state,
        double _max_curvature, double _max_sigma) :
        hc_rs_space(_max_curvature, _max_sigma) {
        start_state = State {
            _start_state.x,
            _start_state.y,
            _start_state.a,
            _start_state.curvature,
            _start_state.is_backward ? -1.0 : 1.0 };

        State end_state = State {
            _end_state.x,
            _end_state.y,
            _end_state.a,
            _end_state.curvature,
            _end_state.is_backward ? -1.0 : 1.0 };

        controls = hc_rs_space.get_controls(start_state, end_state);
    }

    double PathPlanner::hc_reeds_shepp_path_provider::get_length() const {
        return _get_length(controls); }

    bool PathPlanner::hc_reeds_shepp_path_provider::get_is_map_exceeded() const {
        return _get_is_map_exceeded(hc_rs_space, start_state, controls); }

    bool PathPlanner::hc_reeds_shepp_path_provider::traverse(
        const function<bool (const astate&)>& callback) const {
        return _traverse(hc_rs_space, start_state, controls,
            callback, ANALYTIC_EXPANSION_SAMPLING_STEP);
    }
}