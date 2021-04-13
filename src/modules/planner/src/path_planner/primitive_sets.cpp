#include "path_planner.h"
#include "math.h"

namespace TiEV {

constexpr double curvatures[] = {
    0.1818181818, 0.13437, 0.03939, 0.01922, 0.01155, 0.00778, 0.0056, 0.00429
};
constexpr int curvature_num = sizeof(curvatures) / sizeof(curvatures[0]);

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

    PathPlanner::clothoid_base_primitive_set::clothoid_base_primitive_set() {
        #define def_clothoid(start_k, end_k, length_m, is_backward)\
            primitives.emplace_back(\
                start_k * GRID_RESOLUTION,\
                end_k * GRID_RESOLUTION,\
                length_m / GRID_RESOLUTION,\
                is_backward)
        vector<double> forward_curvatures(begin(curvatures), end(curvatures));
        forward_curvatures.emplace_back(0.0);
        for (int i = curvature_num - 1; i >= 0; --i)
            forward_curvatures.emplace_back(-curvatures[i]);
        for (int i = 0; i < forward_curvatures.size(); ++i) {
            for (int di = 2; di >= -2; --di) {
                int ti = i + di;
                if (ti >= 0 || ti < forward_curvatures.size()) {
                    def_clothoid(
                        forward_curvatures[i],
                        forward_curvatures[ti],
                        3.0, false);
                }
            }
        }

        nexts.resize(primitives.size());
        for (int i = 0; i < primitives.size(); ++i) {
            for (int j = 0; j < primitives.size(); ++j) {
                if (primitives[i].get_end_curvature() ==
                    primitives[j].get_begin_curvature()) {
                    nexts[i].emplace_back(&primitives[j]);
                }
            }
        }

        #undef def_clothoid
    }

    const vector<const PathPlanner::base_primitive*>&
        PathPlanner::clothoid_base_primitive_set::get_nexts(
            const astate& state) const {
        int min_idx = 0;
        double min_diff = abs(state.curvature -
            primitives[0].get_begin_curvature()) +
            (state.is_backward ^ primitives[0].get_states().front().is_backward) * 10;
        for (int i = 1; i < primitives.size(); ++i) {
            double diff = abs(state.curvature -
                primitives[i].get_begin_curvature()) +
                (state.is_backward ^ primitives[i].get_states()
                    .front().is_backward) * 10;
            if (diff < min_diff) {
                min_diff = diff;
                min_idx = i;
            }
        }
        return nexts[min_idx];
    }

    const vector<const PathPlanner::base_primitive*>& PathPlanner::
        clothoid_base_primitive_set::get_nexts(const primitive& primitive) const {
        return nexts[((const clothoid_base_primitive*)primitive.get_base()) - &primitives[0]];
    }
}