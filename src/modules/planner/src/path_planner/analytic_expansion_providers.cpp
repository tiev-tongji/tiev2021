#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"

namespace TiEV {
    PathPlanner::dubins_provider::dubins_provider(
        const astate& _start_state, const astate& _end_state,
        double _curvature, double _step) :
        start_state(_start_state), end_state(_end_state),
        curvature(_curvature), step(_step) {
        if (start_state.is_backward) {
            dubins_shortest_path(&target_path,
                &end_state.x, &start_state.x, 1.0 / curvature);
        } else {
            dubins_shortest_path(&target_path,
                &start_state.x, &end_state.x, 1.0 / curvature);
        }

        target_length = dubins_path_length(&target_path);
        current_step = 0.0;
    }

    bool PathPlanner::dubins_provider::get_next_state(astate& output_state) {
        if (current_step <= target_length) {
            if (start_state.is_backward)
                dubins_path_sample(&target_path,
                    target_length - current_step,
                    &output_state.x);
            else
                dubins_path_sample(&target_path,
                    current_step, &output_state.x);
            output_state.s = start_state.s + current_step;
            output_state.is_backward = start_state.is_backward;
            output_state.curvature = ((last_a == output_state.a) ? 0.0 :
                    (last_a > output_state.a ? 0.04 : -0.04));
            last_a = output_state.a;

            if (current_step != target_length) {
                current_step += step;
                current_step = min(current_step, target_length);
            } else current_step = INFINITY;
            return true;
        } else return false;
    }

    PathPlanner::reeds_shepp_provider::reeds_shepp_provider(
        const astate& _start_state, const astate& _end_state,
        double _curvature, double _step) :
        start_state(_start_state), end_state(_end_state),
        curvature(_curvature), step(_step),
        rs_space(1.0 / _curvature) {
        if (start_state.is_backward) {
            target_path = rs_space.reedsShepp(&end_state.x, &start_state.x);
        } else {
            target_path = rs_space.reedsShepp(&start_state.x, &end_state.x);
        }

        target_length = target_path.length() * rs_space.rho_;
        current_step = 0.0;
    }

    bool PathPlanner::reeds_shepp_provider::get_next_state(astate& output_state) {
        if (current_step <= target_length) {
            if (start_state.is_backward)
                rs_space.interpolate(
                    &end_state.x,
                    target_path,
                    (target_length - current_step) / rs_space.rho_,
                    &output_state.x);
            else
                rs_space.interpolate(
                    &start_state.x,
                    target_path,
                    current_step / rs_space.rho_,
                    &output_state.x);
            output_state.s = start_state.s + current_step;
            output_state.is_backward = start_state.is_backward;
            output_state.curvature = ((last_a == output_state.a) ? 0.0 :
                    (last_a > output_state.a ? 0.04 : -0.04));
            last_a = output_state.a;

            if (current_step != target_length) {
                current_step += step;
                current_step = min(current_step, target_length);
            } else current_step = INFINITY;
            return true;
        } else return false;
    }
}