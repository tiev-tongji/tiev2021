#include "Clothoid.hh"
#include "math.h"
#include "path_planner.h"
#include "tievlog.h"

namespace TiEV {
PathPlanner::base_primitive::base_primitive(
    const vector<astate>& _sampled_states)
    : sampled_states(_sampled_states) {
  assert(!_sampled_states.empty());

  max_curvature     = 0.0;
  average_curvature = 0.0;
  is_backward       = false;
  for (const auto& state : _sampled_states) {
    max_curvature = max(max_curvature, fabs(state.curvature));
    average_curvature += state.curvature;
    is_backward |= state.is_backward;
  }

  average_curvature /= _sampled_states.size();
  length = _sampled_states.back().s - _sampled_states.front().s;
}

vector<PathPlanner::astate> PathPlanner::base_primitive::generate_line(
    double length, bool is_backward) {
  vector<astate> sampled_states;
  sampled_states.reserve((int)(length / PRIMITIVE_SAMPLING_STEP) + 1);
  astate tmp_state = {0.0, 0.0, 0.0, 0.0, 0.0, is_backward};
  double delta_x =
      is_backward ? -PRIMITIVE_SAMPLING_STEP : PRIMITIVE_SAMPLING_STEP;
  while (tmp_state.s <= length + PRIMITIVE_SAMPLING_STEP / 2) {
    sampled_states.push_back(tmp_state);
    tmp_state.x += delta_x;
    tmp_state.s += PRIMITIVE_SAMPLING_STEP;
  }
  return sampled_states;
}

vector<PathPlanner::astate> PathPlanner::base_primitive::generate_arc(
    double curvature, double length, bool is_backward) {
  vector<astate> sampled_states;
  sampled_states.reserve((int)(length / PRIMITIVE_SAMPLING_STEP + 0.5) + 1);
  if (curvature == 0.0) {
    cerr << "generate_arc requires a non-zero curvature, or you should use a "
            "line_base_primitive."
         << endl;
    throw exception();
  }

  double radius   = fabs(1.0 / curvature);
  bool   is_left  = curvature > 0.0;
  double center_x = 0, center_y = radius;
  double vec_x = 0, vec_y = -radius;
  double delta_alpha   = PRIMITIVE_SAMPLING_STEP / radius;
  double max_abs_alpha = length / radius + delta_alpha / 2;

  astate tmp_state;
  tmp_state.is_backward = is_backward;
  tmp_state.curvature   = 1.0 / radius;
  tmp_state.s           = 0.0;

  if (!is_left) {
    center_y            = -center_y;
    vec_y               = -vec_y;
    delta_alpha         = -delta_alpha;
    tmp_state.curvature = -tmp_state.curvature;
  }
  if (is_backward) delta_alpha = -delta_alpha;

  for (double alpha = 0.0; fabs(alpha) <= max_abs_alpha; alpha += delta_alpha) {
    double sin_alpha = sin(alpha);
    double cos_alpha = cos(alpha);
    double sampled_x = cos_alpha * vec_x - sin_alpha * vec_y + center_x;
    double sampled_y = sin_alpha * vec_x + cos_alpha * vec_y + center_y;

    tmp_state.x = sampled_x;
    tmp_state.y = sampled_y;
    tmp_state.a = alpha;

    sampled_states.push_back(tmp_state);

    tmp_state.s += PRIMITIVE_SAMPLING_STEP;
  }

  return sampled_states;
}

vector<PathPlanner::astate> PathPlanner::base_primitive::generate_clothoid(
    double begin_curvature, double k_step, bool is_backward) {
  vector<astate> sampled_states;
  int            npts = (int)(primi_l / PRIMITIVE_SAMPLING_STEP + 0.5) + 1;
  sampled_states.reserve(npts);
  double                  theta_0 = is_backward ? M_PI : 0.0;
  Clothoid::ClothoidCurve clothoid_curve(0.0, 0.0, theta_0, begin_curvature,
                                         k_step, primi_l);
  astate tmp_state{0.0, 0.0, 0.0, 0.0, begin_curvature, is_backward};
  while (primi_l + PRIMITIVE_SAMPLING_STEP / 2 >= tmp_state.s) {
    clothoid_curve.eval(tmp_state.s, tmp_state.a, tmp_state.curvature,
                        tmp_state.x, tmp_state.y);
    if (is_backward) tmp_state.a = fmod(tmp_state.a + M_PI, 2 * M_PI);
    sampled_states.push_back(tmp_state);
    tmp_state.s += PRIMITIVE_SAMPLING_STEP;
  }
  return sampled_states;
}

PathPlanner::arc_base_primitive::arc_base_primitive(double _curvature,
                                                    double _length,
                                                    bool   _is_backward)
    : base_primitive(_curvature == 0.0
                         ? generate_line(_length, _is_backward)
                         : generate_arc(_curvature, _length, _is_backward)),
      curvature(_curvature) {}

PathPlanner::line_base_primitive::line_base_primitive(double _length,
                                                      bool   _is_backward)
    : base_primitive(generate_line(_length, _is_backward)) {}

PathPlanner::clothoid_base_primitive::clothoid_base_primitive(
    double _begin_curvature, double _k_step, double _length, bool _is_backward)
    : base_primitive(
          _is_backward
              ? (_k_step != 0.0 ? generate_arc(_k_step, primi_l, _is_backward)
                                : generate_line(primi_l, _is_backward))
              : generate_clothoid(_begin_curvature, _k_step, _is_backward)),
      begin_curvature(_begin_curvature),
      end_curvature(_begin_curvature + _k_step) {}

PathPlanner::primitive::primitive(const base_primitive& _base,
                                  const primitive_ptr   _parent)
    : base(_base), parent(_parent) {}

const vector<PathPlanner::astate>& PathPlanner::primitive::get_states() {
  return base.get_states();
}

PathPlanner::astate PathPlanner::primitive::get_start_state() const {
  return base.get_states().front();
}

PathPlanner::astate PathPlanner::primitive::get_end_state() const {
  return base.get_states().back();
}

double PathPlanner::primitive::get_length() const { return base.get_length(); }

const PathPlanner::base_primitive& PathPlanner::primitive::get_base() const {
  return base;
}

const PathPlanner::primitive_ptr PathPlanner::primitive::get_parent() const {
  return parent;
}

}  // namespace TiEV