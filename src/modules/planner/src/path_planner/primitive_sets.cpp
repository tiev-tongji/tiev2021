#include "path_planner.h"
#include "math.h"
#include "log.h"

namespace TiEV {

#define ITERATE_VECTOR(key_name, value_name, vector, SEGMENT)\
    for (int key_name = 0, _vec_size_ = vector.size();\
        key_name < _vec_size_; ++key_name)\
        do { auto& value_name = vector[key_name]; SEGMENT } while(0)

constexpr double curvatures[] = { 0.1818181818, 0.13437, 0.03939, 0.01922, 0.01155, 0.00778, 0.0056, 0.00429 };
constexpr int curvature_num = sizeof(curvatures) / sizeof(curvatures[0]);
constexpr double max_curvature_sharpness = 0.01;

    PathPlanner::arc_base_primitive_set::arc_base_primitive_set() {
        // generate arcs
        #define def_arc(curvature_m, length_m, backward)\
            primitives.emplace_back(\
            (curvature_m * GRID_RESOLUTION),\
            (length_m / GRID_RESOLUTION),\
            backward)

        vector<double> arc_curvatures(begin(curvatures), end(curvatures));
        arc_curvatures.emplace_back(0.0);
        for (int i = curvature_num - 1; i >= 0; --i)
            arc_curvatures.emplace_back(-curvatures[i]);
        for (int i = 0; i < arc_curvatures.size(); ++i)
            def_arc(arc_curvatures[i], 3.0, false);
        int line_idx = arc_curvatures.size() / 2;
        for (int i = 0; i < arc_curvatures.size(); ++i) {
            if (abs(i - line_idx) <= 3) {
                def_arc(arc_curvatures[i], 3.0, true);
            }
        }

        nexts.resize(primitives.size());
        for (int i = 0; i < primitives.size(); ++i) {
            for (int j = 0; j < primitives.size(); ++j) {
                if (primitives[i].get_is_backward() ==
                    primitives[j].get_is_backward()) {
                    if (abs(i - j) <= 2)
                        nexts[i].emplace_back(&primitives[j]);
                } else {
                    if (fabs(primitives[i].get_curvature()) ==
                        fabs(primitives[j].get_curvature()))
                        nexts[i].emplace_back(&primitives[j]);
                }
            }
        }

        #undef def_arc
    }

    const vector<const PathPlanner::base_primitive*>&
        PathPlanner::arc_base_primitive_set::get_nexts(const astate& state) const {
        int min_idx = 0;
        double min_diff = abs(state.curvature - primitives[0].get_curvature()) +
            (state.is_backward ^ primitives[0].get_is_backward()) * 10;
        for (int i = 1; i < primitives.size(); ++i) {
            double diff = abs(state.curvature - primitives[i].get_curvature()) +
                (state.is_backward ^ primitives[i].get_is_backward()) * 10;
            if (diff < min_diff) {
                min_diff = diff;
                min_idx = i;
            }
        }
        return nexts[min_idx];
    }

    const vector<const PathPlanner::base_primitive*>&
        PathPlanner::arc_base_primitive_set::get_nexts(const primitive& primitive) const {
        return nexts[((const arc_base_primitive*)primitive.get_base()) - &primitives[0]];
    }

    void PathPlanner::clothoid_base_primitive_set::
        generate_clothoid_base_primitive_set(
            double max_curvature, double max_sigma,
            double clothoid_length, bool backward_enabled,
            vector<clothoid_base_primitive>& out_primitives,
            vector<vector<const base_primitive*>>& out_nexts) {

        out_primitives.clear();
        out_nexts.clear();

        #define def_clothoid(start_k, end_k, length_m, is_backward) do {\
            out_primitives.emplace_back(start_k * GRID_RESOLUTION,\
                end_k * GRID_RESOLUTION, length_m / GRID_RESOLUTION, is_backward);\
            log_2("generating clothoid from k = ", start_k, " to k = ", end_k,\
                " in ", length_m, " m, driving ", is_backward ? "backward" : "forward");\
            } while (0)
        double max_curvature_change = (clothoid_length * max_sigma);

        // forward_sampling
        constexpr int forward_sampling_num = 6;
        constexpr double forward_sampling_param = 1.0 / pow(forward_sampling_num - 1, 2);
        vector<double> forward_curvatures;
        for (int i = -(forward_sampling_num - 1); i < forward_sampling_num; ++i)
            forward_curvatures.push_back(i * abs(i) * forward_sampling_param * max_curvature);
        ITERATE_VECTOR(i1, k1, forward_curvatures, {
            ITERATE_VECTOR(i2, k2, forward_curvatures, {
                if (abs(i1 - i2) <= 1 || fabs(k1 - k2) <= max_curvature_change)
                    def_clothoid(k1, k2, clothoid_length, false);
            });
        });

        if (backward_enabled) {
            // backward_sampling
            constexpr int backward_sampling_num = 2;
            constexpr double backward_sampling_param = 1.0 / pow(backward_sampling_num - 1, 2);
            vector<double> backward_curvatures;
            for (int i = -(backward_sampling_num - 1); i < backward_sampling_num; ++i)
                backward_curvatures.push_back(i * abs(i) * backward_sampling_param * max_curvature);
            ITERATE_VECTOR(i1, k1, backward_curvatures, {
                ITERATE_VECTOR(i2, k2, backward_curvatures, {
                    if (abs(i1 - i2) <= 1 || fabs(k1 - k2) <= max_curvature_change)
                        def_clothoid(k1, k2, clothoid_length, true);
                });
            });
        }

        log_2("generating nexts vector");
        out_nexts.resize(out_primitives.size());

        ITERATE_VECTOR(i, pi, out_primitives, {
            ITERATE_VECTOR(j, pj, out_primitives, {
                if (pi.get_is_backward() == pj.get_is_backward()) {
                    if (pi.get_end_curvature() == pj.get_begin_curvature())
                        out_nexts[i].emplace_back(&pj);
                } else out_nexts[i].emplace_back(&pj);
            });
        });

        #undef def_clothoid
    }

    PathPlanner::clothoid_base_primitive_set::clothoid_base_primitive_set() {
        log_0("constructing ", nameof(clothoid_base_primitive_set));
        constexpr double min_speed_km_h = 0;
        constexpr double max_speed_km_h = 80;
        constexpr double max_speed_backward_km_h = 20;
        constexpr int speed_sampling_cnt = 9;
        constexpr double speed_sampling_step =
            (max_speed_km_h - min_speed_km_h) / (speed_sampling_cnt - 1);

        // forward and backward subsets
        subsets.reserve(speed_sampling_cnt * 2);
        for (int i = 0; i < speed_sampling_cnt; ++i) {
            double speed_km_h = min_speed_km_h + i * speed_sampling_step;
            double speed_m_s = speed_km_h / 3.6;
            double speed_descent_m_s = (CAR_CEN_ROW * GRID_RESOLUTION *
                PathPlanner::SPEED_DESCENT_FACTOR) / speed_m_s;
            double min_possible_speed_m_s = max(speed_m_s - speed_descent_m_s, 0.0);
            double max_curvature = max_curvature_under_velocity(min_possible_speed_m_s);
            double max_sigma = max_sigma_under_velocity(speed_m_s);
            double clothoid_length = 4.0 + 6.0 / (speed_sampling_cnt - 1.0) * i;
            subsets.push_back({speed_m_s, false, {}, {}});
            generate_clothoid_base_primitive_set(
                max_curvature, max_sigma, clothoid_length,
                subsets.back().backward_enabled,
                subsets.back().primitives,
                subsets.back().nexts);
            if (min_possible_speed_m_s == 0.0) {
                subsets.push_back({speed_m_s, true, {}, {}});
                generate_clothoid_base_primitive_set(
                    max_curvature, max_sigma, clothoid_length,
                    subsets.back().backward_enabled,
                    subsets.back().primitives,
                    subsets.back().nexts);
            }
        }
    }

    void PathPlanner::clothoid_base_primitive_set::prepare(double current_speed_m_s, bool backward_enabled) {
        double min_speed_delta = numeric_limits<double>::max();
        for (const auto& subset : subsets) {
            if (subset.backward_enabled != backward_enabled) continue;
            if (subset.current_speed_m_s > current_speed_m_s) continue;
            double delta = current_speed_m_s - subset.current_speed_m_s;
            if (delta < min_speed_delta) {
                min_speed_delta = delta;
                current_subset = &subset;
            }
        }
        log_1(nameof(clothoid_base_primitive_set), " prepared for current speed ",
            current_speed_m_s, " m/s, using subset with speed = ",
            current_subset->current_speed_m_s, " m/s, ",
            (backward_enabled ? "backwarding" : "forwarding"));
    }

    const vector<const PathPlanner::base_primitive*>&
        PathPlanner::clothoid_base_primitive_set::get_nexts(const astate& state) const {
        int min_idx;
        double min_diff;
        for (const auto& prim : current_subset->primitives) {
            if (prim.get_states().back().is_backward != state.is_backward) continue;
            double diff = fabs(state.curvature - prim.get_end_curvature());
            if (diff < min_diff) {
                min_diff = diff;
                min_idx = &prim - &current_subset->primitives[0];
            }
        }
        return current_subset->nexts[min_idx];
    }

    const vector<const PathPlanner::base_primitive*>& PathPlanner::
        clothoid_base_primitive_set::get_nexts(const primitive& primitive) const {
        return current_subset->nexts[((const clothoid_base_primitive*)primitive.get_base()) - &current_subset->primitives[0]];
    }
}