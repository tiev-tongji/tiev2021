#include "path_planner.h"
#include "look_up_tables/dubins_table/dubins.h"

namespace TiEV {
    void PathPlanner::dubins_provider::prepare(const astate& _start_state,
        const astate& _end_state, double _max_curvature) {
        start_state = _start_state;
        end_state = _end_state;
        max_curvature = _max_curvature;
        if (start_state.is_backward)
            dubins_shortest_path(&target_path, &end_state.x,
                &start_state.x, 1.0 / max_curvature);
        else
            dubins_shortest_path(&target_path, &start_state.x,
                &end_state.x, 1.0 / max_curvature);
        target_length = dubins_path_length(&target_path);
        current_step = 0;
        total_steps = (int)floor(1.0 + target_length /
            ANALYTIC_EXPANSION_SAMPLING_STEP);
    }

    bool PathPlanner::dubins_provider::get_next_state(astate& output_state) {
        if (current_step < total_steps) {
            double current_s = min(target_length, current_step *
                ANALYTIC_EXPANSION_SAMPLING_STEP);
            if (start_state.is_backward)
                dubins_path_sample(&target_path,
                    target_length - current_s,
                    &output_state.x);
            else
                dubins_path_sample(&target_path,
                    current_s, &output_state.x);
            output_state.s = start_state.s + current_s;
            output_state.is_backward = start_state.is_backward;
            output_state.curvature = ((last_a == output_state.a) ? 0.0 :
                    (last_a > output_state.a ? 0.04 : -0.04));
            last_a = output_state.a;
            ++current_step;
            return true;
        } else return false;
    }

    void PathPlanner::reeds_shepp_provider::prepare(const astate& _start_state,
        const astate& _end_state, double _max_curvature) {
        start_state = _start_state;
        end_state = _end_state;
        max_curvature = _max_curvature;
        rs_space = ReedsSheppStateSpace(1.0 / _max_curvature);

        if (start_state.is_backward) {
            target_path = rs_space.reedsShepp(&end_state.x, &start_state.x);
        } else {
            target_path = rs_space.reedsShepp(&start_state.x, &end_state.x);
        }

        target_length = target_path.length() * rs_space.rho_;
        current_step = 0;
        total_steps = (int)floor(1.0 + target_length /
            ANALYTIC_EXPANSION_SAMPLING_STEP);
    }

    bool PathPlanner::reeds_shepp_provider::get_next_state(astate& output_state) {
        if (current_step < total_steps) {
            double current_s = min(target_length, current_step *
                ANALYTIC_EXPANSION_SAMPLING_STEP);
            if (start_state.is_backward)
                rs_space.interpolate(
                    &end_state.x,
                    target_path,
                    (target_length - current_s) / rs_space.rho_,
                    &output_state.x);
            else
                rs_space.interpolate(
                    &start_state.x,
                    target_path,
                    current_s / rs_space.rho_,
                    &output_state.x);
            output_state.s = start_state.s + current_s;
            output_state.is_backward = start_state.is_backward;
            output_state.curvature = ((last_a == output_state.a) ? 0.0 :
                    (last_a > output_state.a ? 0.04 : -0.04));
            last_a = output_state.a;

            ++current_step;
            return true;
        } else return false;
    }
}