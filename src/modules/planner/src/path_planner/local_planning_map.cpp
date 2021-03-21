#include "path_planner.h"
#include "math.h"
#include "collision_check.h"
#include "log.h"

namespace TiEV {
    constexpr double len(int a, int b) {
        return sqrt(a * a + b * b);
    }

    void PathPlanner::local_planning_map::init(
        const astate& _target,
        double (*_safe_map)[MAX_COL],
        bool _backward_enabled,
        double _backward_cost_factor) {
        target = _target;
        safe_map = _safe_map;
        backward_enabled = _backward_enabled;
        backward_cost_factor = _backward_cost_factor;

        time_t time_1 = getTimeStamp();
        calculate_xya_distance_map();
        time_t time_2 = getTimeStamp();
        log(1, "xya_distance_map calculated in ", (time_2 - time_1) / 1000, " ms");
    }

    bool PathPlanner::local_planning_map::is_crashed(int x, int y) const {
        return safe_map[x][y] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION;
    }

    bool PathPlanner::local_planning_map::is_crashed(const astate& state) const {
        return collision(state.x, state.y, state.a, safe_map, 0.0);
    }

    bool PathPlanner::local_planning_map::is_crashed(primitive& prim) const {
        if (!is_in_map(prim.get_end_state())) return true;
        double safe_distance = get_maximum_safe_distance(
            prim.get_start_state());
        if (prim.get_length() < safe_distance) return false;
        for (const auto& state : prim.get_states()) {
            if (state.s <= safe_distance) continue;
            else if (is_crashed(state)) return true;
            else safe_distance = state.s +
                get_maximum_safe_distance(state);
        }
        return false;
    }

    double PathPlanner::local_planning_map::get_heuristic(
        const astate& state, bool can_reverse) const {
        const int x_idx = (int)round(state.x);
        const int y_idx = (int)round(state.y);
        const int xya_idx_x = x_idx >> XYA_MAP_SHIFT_FACTOR;
        const int xya_idx_y = y_idx >> XYA_MAP_SHIFT_FACTOR;
        const int xya_idx_a = get_angle_index(state.a);

        return xya_distance_map[xya_idx_x][xya_idx_y][xya_idx_a];
    }

    int PathPlanner::local_planning_map::try_get_target_index(
        primitive& primitive) const {
        const auto& start_state = primitive.get_start_state();
        double dis = max(
            abs(start_state.x - target.x),
            abs(start_state.y - target.y)
        );

        if (dis <= primitive.get_length()) {
            int index = 0;
            for (const auto& state : primitive.get_states())
                if (is_target(state)) return index;
                else ++ index;
        }

        return -1;
    }

    bool PathPlanner::local_planning_map::is_in_map(const astate& state) const {
        return state.x >= 0 && state.y >= 0 &&
            state.x <= (MAX_ROW - 1) && state.y <= (MAX_COL - 1);
    }

    void PathPlanner::local_planning_map::merge_xya_distance_map(
        pair<double, double> (*output_map)[MAX_COL]) const {
        for (int i = 0; i < MAX_ROW; ++i)
            for (int j = 0; j < MAX_COL; ++j) {
                output_map[i][j].first = 1e9;
                const double* a_vec = xya_distance_map[i >> XYA_MAP_SHIFT_FACTOR]
                        [j >> XYA_MAP_SHIFT_FACTOR];
                for (int a = 0; a < XYA_MAP_DEPTH; ++a) {
                    if (a_vec[a] < output_map[i][j].first) {
                        output_map[i][j].first = a_vec[a];
                        output_map[i][j].second = M_PI - (a + 0.5) * XYA_MAP_DELTA_A;
                    }
                }
            }
    }

    bool PathPlanner::local_planning_map::is_target(const astate& state) const {
        constexpr double dx = 1;
        constexpr double dy = 1;
        constexpr double da = 3 / 180.0 * M_PI;
        if (fabs(state.x - target.x) > dx) return false;
        if (fabs(state.y - target.y) > dy) return false;
        if (fabs(state.a - target.a) > da) return false;
        return true;
    }

    int PathPlanner::local_planning_map::get_angle_index(double ang) {
        double norm_ang = M_PI - ang;
        norm_ang -= floor(norm_ang / (2 * PI)) * (2 * PI);
        return (int)(norm_ang / XYA_MAP_DELTA_A) % XYA_MAP_DEPTH;
    }

    void PathPlanner::local_planning_map::
        calculate_xya_distance_map() {
        // calculate xya_safe_map by counting safe and unsafe small grid
        // cells in each xya big grid cell. if safe cells are more then
        // the big grid is considered safe.
        memset(xya_safe_map, 0, sizeof(xya_safe_map));
        for (int i = 0; i < XYA_MAP_ROWS - 1; ++i)
            for (int j = 0; j < XYA_MAP_COLS - 1; ++j) {
                int idx = (i << XYA_MAP_SHIFT_FACTOR);
                int jdx = (j << XYA_MAP_SHIFT_FACTOR);
                int safe_factor = 0;
                for (int di = 0; di < (1 << XYA_MAP_SHIFT_FACTOR); ++di)
                    for (int dj = 0; dj < (1 << XYA_MAP_SHIFT_FACTOR); ++dj)
                        if (is_crashed(idx + di, jdx + dj)) --safe_factor;
                        else ++safe_factor;
                xya_safe_map[i][j] = safe_factor >= 0;
            }

        #define vec(a, b) { a * (XYA_MAP_COLS * XYA_MAP_DEPTH) + b * XYA_MAP_DEPTH, (1 << XYA_MAP_SHIFT_FACTOR) * len(a, b) }
        constexpr pair<int, double> deltas[] = {
            vec( 1, 0), vec( 1,-1), vec( 0,-1), vec(-1,-1),
            vec(-1, 0), vec(-1, 1), vec( 0, 1), vec( 1, 1)
        };
        #undef vec
        constexpr int DELTAS_LENGTH = sizeof(deltas) / sizeof(pair<int, double>);
        constexpr int DEPTHS_PER_DELTA = XYA_MAP_DEPTH / DELTAS_LENGTH;
        // https://i.loli.net/2021/03/14/TKVI3kL6mtXcONv.png
        #define get_delta_idx(a_idx) (a_idx + (DEPTHS_PER_DELTA / 2)) % XYA_MAP_DEPTH / DEPTHS_PER_DELTA

        // we embedded the x and y value as one integer
        // and add it with deltas[i].first to do the translation
        // this is far more faster than using x and y separately
        double* flatten_map = (double*)xya_distance_map;
        bool* flatten_safe_map = (bool*)xya_safe_map;
        ring_buffer_xya.clear();
        memset(is_in_buffer_xya, 0, sizeof(is_in_buffer_xya));
        memset(xya_distance_map, 0x7f, sizeof(xya_distance_map));

        int target_x_idx = (int)round(target.x) >> XYA_MAP_SHIFT_FACTOR;
        int target_y_idx = (int)round(target.y) >> XYA_MAP_SHIFT_FACTOR;
        int target_a_idx = get_angle_index(target.a);
        int target_xya = target_x_idx * (XYA_MAP_COLS * XYA_MAP_DEPTH) +
            target_y_idx * XYA_MAP_DEPTH + target_a_idx;

        flatten_map[target_xya] = 0.0;
        ring_buffer_xya.push_back(target_xya);
        is_in_buffer_xya[target_xya] = true;

        while(!ring_buffer_xya.empty()) {
            int xya = ring_buffer_xya.front();
            ring_buffer_xya.pop_front();
            is_in_buffer_xya[xya] = false;
            int a = xya % XYA_MAP_DEPTH;
            int xy0 = xya - a;
            int delta_idx = get_delta_idx(a);

            for (int opposite = 0; opposite <= backward_enabled; ++opposite) {
                int zw0 = xy0;
                if (opposite) zw0 -= deltas[delta_idx].first;
                else zw0 += deltas[delta_idx].first;

                if (zw0 < 0 || !flatten_safe_map[zw0 / XYA_MAP_DEPTH]) continue;

                double new_dis = flatten_map[xya] + deltas[delta_idx].second;
                if (opposite) new_dis += backward_cost_factor * deltas[delta_idx].second;

                for (int da = -1; da <= 1; ++da) {
                    int s = (a + da + XYA_MAP_DEPTH) % XYA_MAP_DEPTH;
                    int zws = zw0 + s;
                    if (flatten_map[zws] > new_dis) {
                        flatten_map[zws] = new_dis;
                        if (!is_in_buffer_xya[zws]) {
                            ring_buffer_xya.push_back(zws);
                            is_in_buffer_xya[zws] = true;
                        }
                    }
                }
            }
        }

        // if one cell in xya_distance_map is not safe, but
        // a neighboor of it is safe, we simply copy the neighboor's
        // distances to it. this ensures the search to get heuristic
        // even at narrow places. this will not cause unsafe factor,
        // since we only use xya_distance_map for heuristic, and still
        // do collision check on the original safe map.
        for (int i = 0; i < XYA_MAP_ROWS - 1; ++i)
            for (int j = 0; j < XYA_MAP_COLS - 1; ++j) {
                int ij = i * (XYA_MAP_COLS) + j;
                if (flatten_safe_map[ij]) continue;
                for (int k = 0; k < DELTAS_LENGTH; ++k) {
                    int ninj = ij + deltas[k].first / XYA_MAP_DEPTH;
                    if (ninj < 0) continue;
                    if (flatten_safe_map[ninj]) {
                        memcpy(flatten_map + ij * XYA_MAP_DEPTH,
                            flatten_map + ninj * XYA_MAP_DEPTH,
                            sizeof(double) * XYA_MAP_DEPTH);
                        break;
                    }
                }
            }
        #undef get_delta_idx
    }

    double PathPlanner::local_planning_map::
        get_maximum_safe_distance(const astate& state) const {
        int dis_x = (int)round(state.x), dis_y = (int)round(state.y);
        double dis = safe_map[dis_x][dis_y] - sqrt(2);
        return dis - COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION;
    }
}