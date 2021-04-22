#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"

namespace TiEV {
    inline double _get_length(const std::vector<steer::Control>& controls) {
        double l = 0.0;
        for (const auto& control : controls)
            l += fabs(control.delta_s);
        return l;
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

    void PathPlanner::dubins_provider::sample(double t, astate& output_state) const {
        State state = dubins_space.interpolate(start_state, controls, t);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;
    }

    double PathPlanner::dubins_provider::get_length() const { return _get_length(controls); }

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

    void PathPlanner::reeds_shepp_provider::sample(double t, astate& output_state) const {
        State state = rs_space.interpolate(start_state, controls, t);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;
    }

    double PathPlanner::reeds_shepp_provider::get_length() const { return _get_length(controls); }

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

    void PathPlanner::cc_dubins_path_provider::sample(double t, astate& output_state) const {
        State state = cc_dubins_space.interpolate(start_state, controls, t);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;
    }

    double PathPlanner::cc_dubins_path_provider::get_length() const { return _get_length(controls); }

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

    void PathPlanner::hc_reeds_shepp_path_provider::sample(double t, astate& output_state) const {
        State state = hc_rs_space.interpolate(start_state, controls, t);
        output_state.x = state.x;
        output_state.y = state.y;
        output_state.a = state.theta;
        output_state.curvature = state.kappa;
        output_state.is_backward = state.d < 0.0;
    }

    double PathPlanner::hc_reeds_shepp_path_provider::get_length() const { return _get_length(controls); }
}