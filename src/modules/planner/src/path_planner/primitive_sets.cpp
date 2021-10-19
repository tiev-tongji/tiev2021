#include "log.h"
#include "math.h"
#include "path_planner.h"
#include "tievlog.h"

namespace TiEV {

void PathPlanner::clothoid_base_primitive_set::
    generate_clothoid_base_primitive_set(
        double begin_k, bool backward_enabled,
        vector<clothoid_base_primitive>& out_primitives) {
  out_primitives.clear();

  const auto k_is_legal = [](double k) {
    return k < -CAR_MAX_K ? false : (k > CAR_MAX_K ? false : true);
  };

  // only forward generate arc for each k_step
  for (const auto& k_step : k_step_list) {
    // for each k_step, we generate the arc as primitive, but the k of arc must
    // be not bigger than max_k
    const auto end_k = begin_k + k_step * primi_l;
    if (!k_is_legal(end_k)) continue;
    out_primitives.emplace_back(begin_k, k_step, primi_l, /*backward=*/false);
  }
  if (backward_enabled) {
    // for backward, we just generate the -max_k, 0 and max_k arcs
    out_primitives.emplace_back(begin_k, CAR_MAX_K, primi_l, backward_enabled);
    out_primitives.emplace_back(begin_k, 0.0, primi_l, backward_enabled);
    out_primitives.emplace_back(begin_k, -CAR_MAX_K, primi_l, backward_enabled);
  }
}

PathPlanner::clothoid_base_primitive_set::clothoid_base_primitive_set() {
  LOG(INFO) << "constructing " << nameof(clothoid_base_primitive_set);
  // forward and backward sets
  all_k_sets.reserve(k_list.size() * 2);
  for (int i = 0; i < k_list.size(); ++i) {
    // only forward primitives
    all_k_sets.push_back({k_list[i], false, {}});
    generate_clothoid_base_primitive_set(all_k_sets.back().current_k,
                                         all_k_sets.back().backward_enabled,
                                         all_k_sets.back().primitives);
    // forward and backward primitives
    all_k_sets.push_back({k_list[i], true, {}});
    generate_clothoid_base_primitive_set(all_k_sets.back().current_k,
                                         all_k_sets.back().backward_enabled,
                                         all_k_sets.back().primitives);
  }
}

void PathPlanner::clothoid_base_primitive_set::prepare(bool backward_enabled) {
  current_subset.clear();
  for (const auto& subset : all_k_sets) {
    // LOG(WARNING) << "subset k:" << subset.current_k;
    // for (const auto& primi : subset.primitives) {
    //   LOG(INFO) << "----primitive----k_step="
    //             << primi.get_end_curvature() - primi.get_begin_curvature();
    //   for (const auto& st : primi.get_states()) {
    //     LOG(INFO) << st;
    //   }
    // }
    if (subset.backward_enabled != backward_enabled) continue;
    current_subset.push_back(&subset);
  }
}

const vector<PathPlanner::base_primitive>
PathPlanner::clothoid_base_primitive_set::get_nexts(
    const astate& state, const double current_speed) const {
  // select the primitives by state's k
  double min_delta_k = numeric_limits<double>::max();
  int    min_idx     = -1;
  for (int i = 0; i < current_subset.size(); ++i) {
    const auto& subset  = *current_subset[i];
    const auto  delta_k = fabs(subset.current_k - state.curvature);
    if (delta_k < min_delta_k) {
      min_delta_k = delta_k;
      min_idx     = i;
    }
  }
  // rotate and translate the primitives
  const double cos_a = std::cos(state.a);
  const double sin_a = std::sin(state.a);
  const auto   rotate_and_translate =
      [&](const PathPlanner::clothoid_base_primitive& primi) {
        std::vector<astate> new_states;
        new_states.reserve(primi.get_states().size());
        for (const auto& sta : primi.get_states()) {
          double new_x = sta.x * cos_a / GRID_RESOLUTION -
                         sta.y * sin_a / GRID_RESOLUTION + state.x;
          double new_y = sta.x * sin_a / GRID_RESOLUTION +
                         sta.y * cos_a / GRID_RESOLUTION + state.y;
          new_states.emplace_back(
              new_x, new_y, PathPlanner::wrap_angle_0_2_PI(sta.a + state.a),
              sta.s + state.s, sta.curvature, sta.is_backward);
        }
        return std::move(PathPlanner::base_primitive(new_states));
      };
  // the result
  std::vector<PathPlanner::base_primitive> base;
  base.reserve(current_subset[min_idx]->primitives.size());
  for (const auto& primi : current_subset[min_idx]->primitives) {
    // if (fabs(primi.get_end_curvature()) >
    //     max_curvature_under_velocity(current_speed))
    //   continue;
    base.push_back(rotate_and_translate(primi));
  }
  return base;
}

const vector<PathPlanner::base_primitive>
PathPlanner::clothoid_base_primitive_set::get_nexts(
    const primitive& primitive) const {
  return get_nexts(primitive.get_end_state(), 0.0);
}
}  // namespace TiEV