#include "path_planner.h"
#include "math.h"

namespace TiEV {

constexpr double radiuses[] = {
    0.18182,
    0.13437,
    0.03939,
    0.01922,
    0.01155,
    0.00778,
    0.00560,
    0.00429,
};

    PathPlanner::arc_base_primitive_set::arc_base_primitive_set() {
        // generate arcs
#define def_arc(curvature_m, length_m, left)\
        arc_base_primitive(\
        (1.0 / GRID_RESOLUTION / curvature_m),\
        left, false, (length_m / GRID_RESOLUTION),\
        (0.2 / GRID_RESOLUTION))
#define def_line(length_m) line_base_primitive(\
        (length_m / GRID_RESOLUTION),\
        (0.2 / GRID_RESOLUTION), false)

        primitives.push_back(def_arc(radiuses[0], 3.0, true));
        primitives.push_back(def_arc(radiuses[1], 3.0, true));
        primitives.push_back(def_arc(radiuses[2], 3.0, true));
        primitives.push_back(def_arc(radiuses[3], 3.0, true));
        primitives.push_back(def_arc(radiuses[4], 3.0, true));
        primitives.push_back(def_arc(radiuses[5], 3.0, true));
        primitives.push_back(def_arc(radiuses[6], 3.0, true));
        primitives.push_back(def_arc(radiuses[7], 3.0, true));
        primitives.push_back(def_line(3.0));
        primitives.push_back(def_arc(radiuses[7], 3.0, false));
        primitives.push_back(def_arc(radiuses[6], 3.0, false));
        primitives.push_back(def_arc(radiuses[5], 3.0, false));
        primitives.push_back(def_arc(radiuses[4], 3.0, false));
        primitives.push_back(def_arc(radiuses[3], 3.0, false));
        primitives.push_back(def_arc(radiuses[2], 3.0, false));
        primitives.push_back(def_arc(radiuses[1], 3.0, false));
        primitives.push_back(def_arc(radiuses[0], 3.0, false));

#undef def_line
#undef def_arc

        // generate nexts
        nexts.resize(primitives.size());
        for (int i = 0; i < primitives.size(); ++i) {
            for (int di = -2; di <= 2; ++di) {
                int next = i + di;
                if (next >= 0 && next < primitives.size())
                    nexts[i].push_back(&primitives[next]);
            }
        }
    }

    const vector<const PathPlanner::base_primitive*>&
        PathPlanner::arc_base_primitive_set::get_nexts(
            const astate& state) const {
        int min_idx = 0;
        double min_diff = abs(state.curvature -
            primitives[0].get_states().front().curvature);
        for (int i = 1; i < primitives.size(); ++i) {
            double diff = abs(state.curvature -
                primitives[i].get_states().front().curvature);
            if (diff < min_diff) {
                min_diff = diff;
                min_idx = i;
            }
        }
        return nexts[min_idx];
    }

    const vector<const PathPlanner::base_primitive*>& PathPlanner::arc_base_primitive_set::get_nexts(const primitive& primitive) const {
        return nexts[primitive.get_base() - &primitives[0]];
    }

    double PathPlanner::arc_base_primitive_set::get_sampling_step_size() const {
        return 0.2 / GRID_RESOLUTION;
    }
}