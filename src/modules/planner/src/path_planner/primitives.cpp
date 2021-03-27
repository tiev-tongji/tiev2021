#include "path_planner.h"
#include "Clothoid.hh"
#include "math.h"

namespace TiEV {
    PathPlanner::base_primitive::base_primitive(
        const vector<astate>& _sampled_states) :
        sampled_states(_sampled_states) {

        // ensures that sammped states is continuous from (0, 0, 0)
        assert(!_sampled_states.empty());
        assert(_sampled_states.front().x == 0.0);
        assert(_sampled_states.front().y == 0.0);
        assert(_sampled_states.front().a == 0.0);

        max_curvature = 0.0;
        average_curvature = 0.0;
        is_backward = false;
        for (const auto& state : _sampled_states) {
            max_curvature = max(max_curvature, fabs(state.curvature));
            average_curvature += state.curvature;
            is_backward |= state.is_backward;
        }

        average_curvature /= _sampled_states.size();
        length = _sampled_states.back().s;
    }

    vector<PathPlanner::astate> PathPlanner::base_primitive::generate_line(
        double length, bool is_backward) {
        vector<astate> sampled_states;
        sampled_states.reserve((int)(length / PRIMITIVE_SAMPLING_STEP) + 1);
        astate tmp_state = {0.0, 0.0, 0.0, 0.0, 0.0, is_backward};
        double delta_x = is_backward ? -PRIMITIVE_SAMPLING_STEP : PRIMITIVE_SAMPLING_STEP;
        while (tmp_state.s <= length) {
            sampled_states.push_back(tmp_state);
            tmp_state.x += delta_x;
            tmp_state.s += PRIMITIVE_SAMPLING_STEP;
        }
        return sampled_states;
    }

    vector<PathPlanner::astate> PathPlanner::base_primitive::
        generate_arc(double curvature, double length, bool is_backward) {
        vector<astate> sampled_states;
        sampled_states.reserve((int)(length / PRIMITIVE_SAMPLING_STEP) + 1);
        if (curvature == 0.0) {
            cerr << "generate_arc requires a non-zero curvature, or you should use a line_base_primitive." << endl;
            throw exception();
        }

        double radius = fabs(1.0 / curvature);
        bool is_left = curvature > 0.0;
        double center_x = 0, center_y = radius;
        double vec_x = 0, vec_y = -radius;
        double delta_alpha = PRIMITIVE_SAMPLING_STEP / radius;
        double max_abs_alpha = length / radius;

        astate tmp_state;
        tmp_state.is_backward = is_backward;
        tmp_state.curvature = 1.0 / radius;
        tmp_state.s = 0.0;

        if (!is_left) {
            center_y = -center_y;
            vec_y = -vec_y;
            delta_alpha = -delta_alpha;
            tmp_state.curvature = -tmp_state.curvature;
        }
        if (is_backward) delta_alpha = -delta_alpha;

        for (double alpha = 0.0;
            fabs(alpha) <= max_abs_alpha;
            alpha += delta_alpha) {
            double sin_alpha = sin(alpha);
            double cos_alpha = cos(alpha);
            double sampled_x = cos_alpha * vec_x -
                sin_alpha * vec_y + center_x;
            double sampled_y = sin_alpha * vec_x +
                cos_alpha * vec_y + center_y;

            tmp_state.x = sampled_x;
            tmp_state.y = sampled_y;
            tmp_state.a = alpha;

            sampled_states.push_back(tmp_state);

            tmp_state.s += PRIMITIVE_SAMPLING_STEP;
        }

        return sampled_states;
    }

    vector<PathPlanner::astate> PathPlanner::base_primitive::generate_clothoid(
        double begin_curvature, double end_curvature, double length, bool is_backward) {
        vector<astate> sampled_states;
        int npts = (int)(length / PRIMITIVE_SAMPLING_STEP) + 1;
        sampled_states.reserve(npts);
        double dk = (end_curvature - begin_curvature) / length;
        double theta_0 = is_backward ? M_PI : 0.0;
        Clothoid::ClothoidCurve clothoid_curve(
            0.0, 0.0, theta_0, begin_curvature, dk, length);
        astate tmp_state { 0.0, 0.0, 0.0, 0.0, begin_curvature, is_backward };
        while (tmp_state.s <= length) {
            clothoid_curve.eval(tmp_state.s,
                tmp_state.a, tmp_state.curvature, tmp_state.x, tmp_state.y);
            sampled_states.push_back(tmp_state);
            tmp_state.s += PRIMITIVE_SAMPLING_STEP;
        }
        return sampled_states;
    }

    PathPlanner::arc_base_primitive::arc_base_primitive(
        double _curvature, double _length, bool _is_backward) :
        curvature(_curvature), base_primitive(_curvature == 0.0 ?
            generate_line(_length, _is_backward) :
            generate_arc(_curvature, _length, _is_backward)) { }

    PathPlanner::line_base_primitive::line_base_primitive(
        double _length, bool _is_backward) :
        base_primitive(generate_line(_length, _is_backward)) { }

    PathPlanner::clothoid_base_primitive::clothoid_base_primitive(
        double _begin_curvature, double _end_curvature,
        double _length, bool _is_backward) :
        begin_curvature(_begin_curvature), end_curvature(_end_curvature),
        base_primitive(_begin_curvature == _end_curvature ?
            (_begin_curvature == 0.0 ?
                generate_line(_length, _is_backward) :
                generate_arc(_begin_curvature, _length, _is_backward)) :
            generate_clothoid(_begin_curvature, _end_curvature, _length, _is_backward)) { }

    PathPlanner::primitive::primitive(
        const base_primitive* _base,
        const primitive_ptr _parent,
        const astate& _start_state) :
        base(_base),
        parent(_parent),
        sampled(false),
        start_state(_start_state),
        end_state(_base->get_states().back()) {
        // set trans data
        trans_sin = sin(start_state.a);
        trans_cos = cos(start_state.a);
        // compute end_state
        trans(end_state);
    }

    PathPlanner::primitive::primitive(primitive&& _p) :
        base(_p.base), parent(_p.parent),
        start_state(_p.start_state), end_state(_p.end_state),
        sampled_states(move(_p.sampled_states)),
        sampled(_p.sampled),
        trans_sin(_p.trans_sin),
        trans_cos(_p.trans_cos) { }

    const vector<PathPlanner::astate>& PathPlanner::primitive::get_states() {
        if (!sampled) {
            // if not sampled, sample the whole primitive
            sampled_states = base->get_states();
            for (auto& state : sampled_states) trans(state);
            sampled = true;
        }

        return sampled_states;
    }

    PathPlanner::astate PathPlanner::primitive::get_start_state() const {
        return start_state;
    }

    PathPlanner::astate PathPlanner::primitive::get_end_state() const {
        return end_state;
    }

    double PathPlanner::primitive::get_length() const {
        return base->get_length();
    }

    const PathPlanner::base_primitive* PathPlanner::primitive::get_base() const {
        return base;
    }

    const PathPlanner::primitive_ptr PathPlanner::primitive::get_parent() const {
        return parent;
    }

    const bool PathPlanner::primitive::is_samped() const {
        return sampled;
    }

    void PathPlanner::primitive::trans(astate& src) const {
        src.a += start_state.a;
        src.s += start_state.s;
        double new_x = src.x * trans_cos - src.y * trans_sin;
        src.y = src.x * trans_sin + src.y * trans_cos + start_state.y;
        src.x = new_x + start_state.x;
    }
}