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
        double (*_safe_map)[MAX_COL]) {
        target = _target;
        safe_map = _safe_map;

        time_t time_1 = getTimeStamp();
        calculate_xy_distance_map();
        time_t time_2 = getTimeStamp();
        log(1, "xy_distance_map calculated in ", (time_2 - time_1) / 1000, " ms");
        calculate_xya_distance_map();
        time_t time_3 = getTimeStamp();
        log(1, "xya_distance_map calculated in ", (time_3 - time_2) / 1000, " ms");
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

    double PathPlanner::local_planning_map::get_heuristic(const astate& state) const {
        const int x_idx = (int)round(state.x);
        const int y_idx = (int)round(state.y);
        double xy = xy_distance_map[x_idx >> XY_MAP_SHIFT_FACTOR]
            [y_idx >> XY_MAP_SHIFT_FACTOR];
        double xya = xya_distance_map[x_idx >> XYA_MAP_SHIFT_FACTOR]
            [y_idx >> XYA_MAP_SHIFT_FACTOR]
            [get_angle_index(state.a)];
        return max(xy, xya);
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

    void PathPlanner::local_planning_map::merge_xy_distance_map(
        double (*output_map)[MAX_COL]) const {
        for (int i = 0; i < MAX_ROW; ++i)
            for (int j = 0; j < MAX_COL; ++j)
                output_map[i][j] = min(
                    output_map[i][j],
                    xy_distance_map
                        [i >> XY_MAP_SHIFT_FACTOR]
                        [j >> XY_MAP_SHIFT_FACTOR]);
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

    void PathPlanner::local_planning_map::
        calculate_xy_distance_map() {
        memset(xy_safe_map, 0, sizeof(xy_safe_map));
        for (int i = 0; i < MAX_ROW; ++i)
            for (int j = 0; j < MAX_COL; ++j) {
                int iidx = i >> XY_MAP_SHIFT_FACTOR;
                int jidx = j >> XY_MAP_SHIFT_FACTOR;
                if (xy_safe_map[iidx][jidx]) continue;
                else xy_safe_map[iidx][jidx] |= !is_crashed(i, j);
            }

        #define vec(a, b) {{a, b}, (1 << XY_MAP_SHIFT_FACTOR) * len(a, b)}
        constexpr pair<pair<int, int>, double> deltas[] = {
                         vec(-1,  2),              vec( 1,  2),
            vec(-2,  1), vec(-1,  1), vec( 0,  1), vec( 1,  1), vec( 2,  1),
                         vec(-1,  0),              vec( 1,  0),
            vec(-2,  1), vec(-1, -1), vec( 0, -1), vec( 1, -1), vec( 2, -1),
                         vec(-1, -2),              vec( 1, -2)
        };
        #undef vec
        constexpr int num_deltas = sizeof(deltas) /
            sizeof(pair<pair<int, int>, double>);

        ring_buffer_2.clear();
        memset(is_in_buffer_2, 0, sizeof(is_in_buffer_2));
        memset(xy_distance_map, 0x7f, sizeof(xy_distance_map));
        int target_x_idx = (int)round(target.x) >> XY_MAP_SHIFT_FACTOR;
        int target_y_idx = (int)round(target.y) >> XY_MAP_SHIFT_FACTOR;
        xy_distance_map[target_x_idx][target_y_idx] = 0.0;

        ring_buffer_2.push(make_pair(target_x_idx, target_y_idx));
        is_in_buffer_2[target_x_idx][target_y_idx] = true;

        while(!ring_buffer_2.empty()) {
            const int x = ring_buffer_2.front().first;
            const int y = ring_buffer_2.front().second;
            ring_buffer_2.pop();
            is_in_buffer_2[x][y] = false;

            bool check_boundaries = x <= 1 || y <= 1 ||
                x >= (XY_MAP_ROWS - 2) || y >= (XY_MAP_COLS - 2);
            double old_dis = xy_distance_map[x][y];
            for (int i = 0; i < num_deltas; ++i) {
                const int tx = x + deltas[i].first.first;
                const int ty = y + deltas[i].first.second;
                const double delta = deltas[i].second;
                if (check_boundaries && (tx < 0 || ty < 0 ||
                    tx >= XY_MAP_ROWS || ty >= XY_MAP_COLS))
                    continue;
                if (!xy_safe_map[tx][ty]) continue;
                const double new_dis = old_dis + delta;
                if (xy_distance_map[tx][ty] > new_dis) {
                    xy_distance_map[tx][ty] = new_dis;
                    if (!is_in_buffer_2[tx][ty]) {
                        ring_buffer_2.push(make_pair(tx, ty));
                        is_in_buffer_2[tx][ty] = true;
                    }
                }
            }
        }
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
        memset(xya_safe_map, 1, sizeof(xya_safe_map));
        for (int i = 0; i < XYA_MAP_ROWS; ++i)
            for (int j = 0; j < XYA_MAP_COLS; ++j) {
                int idx = (i << XYA_MAP_SHIFT_FACTOR);
                int jdx = (j << XYA_MAP_SHIFT_FACTOR);
                int safe_factor = 0;
                for (int di = 0; di < (1 << XYA_MAP_SHIFT_FACTOR); ++di)
                    for (int dj = 0; dj < (1 << XYA_MAP_SHIFT_FACTOR); ++dj)
                        if (is_crashed(idx + di, jdx + dj)) --safe_factor;
                        else ++safe_factor;
                xya_safe_map[i][j] = safe_factor >= 0;
            }

        #define vec(a, b) {{a, b}, (1 << XYA_MAP_SHIFT_FACTOR) * len(a, b)}
        constexpr pair<pair<int, int>, double> deltas[] = {
            vec( 1, 1), vec( 1, 0), vec( 1,-1), vec( 0,-1),
            vec(-1,-1), vec(-1, 0), vec(-1, 1), vec( 0, 1),
        };
        #undef vec
        constexpr int num_deltas = sizeof(deltas) /
            sizeof(pair<pair<int, int>, double>);
        // https://i.loli.net/2021/03/14/TKVI3kL6mtXcONv.png
        constexpr int start_a_idx = 26;
        #define get_delta_idx(a_idx) \
            ((a_idx) + (XYA_MAP_DEPTH - start_a_idx)) % XYA_MAP_DEPTH / (\
                XYA_MAP_DEPTH / num_deltas);

        ring_buffer_3.clear();
        memset(is_in_buffer_3, 0, sizeof(is_in_buffer_3));
        memset(xya_distance_map, 0x7f, sizeof(xya_distance_map));
        int target_x_idx = (int)round(target.x) >> XYA_MAP_SHIFT_FACTOR;
        int target_y_idx = (int)round(target.y) >> XYA_MAP_SHIFT_FACTOR;
        int target_a_idx = get_angle_index(target.a);
        xya_distance_map[target_x_idx][target_y_idx][target_a_idx] = 0.0;
        ring_buffer_3.push(make_tuple(target_x_idx, target_y_idx, target_a_idx));
        is_in_buffer_3[target_x_idx][target_y_idx][target_a_idx] = true;

        while(!ring_buffer_3.empty()) {
            const auto& front = ring_buffer_3.front();
            const int x = get<0>(front);
            const int y = get<1>(front);
            const int a = get<2>(front);
            const int delta_idx = get_delta_idx(a);
            const int tx = deltas[delta_idx].first.first + x;
            const int ty = deltas[delta_idx].first.second + y;
            const double delta = deltas[delta_idx].second;
            ring_buffer_3.pop();
            is_in_buffer_3[x][y][a] = false;

            if (tx < 0 || ty < 0 || tx >= XYA_MAP_ROWS || ty >= XYA_MAP_COLS
                || !xya_safe_map[tx][ty])
                continue;

            double old_dis = xya_distance_map[x][y][a];
            for (int da = -1; da <= 1; ++da) {
                const int ta = (a + da + XYA_MAP_DEPTH) % XYA_MAP_DEPTH;
                const double new_dis = old_dis + delta;
                if (xya_distance_map[tx][ty][ta] > new_dis) {
                    xya_distance_map[tx][ty][ta] = new_dis;
                    if (!is_in_buffer_3[tx][ty][ta]) {
                        ring_buffer_3.push(make_tuple(tx, ty, ta));
                        is_in_buffer_3[tx][ty][ta] = true;
                    }
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