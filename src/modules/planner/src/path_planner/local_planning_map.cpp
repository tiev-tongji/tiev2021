#include "collision_check.h"
#include "log.h"
#include "math.h"
#include "path_planner.h"

namespace TiEV {

#define MULTILINE(SEGMENT) \
  do {                     \
    SEGMENT                \
  } while (false)
#define BINARY_BRANCH(FLAG, SEGMENT)           \
  MULTILINE(                                   \
      if (FLAG) {                              \
        constexpr bool _is_flag_true_ = true;  \
        SEGMENT                                \
      } else {                                 \
        constexpr bool _is_flag_true_ = false; \
        SEGMENT                                \
      })
#define FLAG                  _is_flag_true_
#define IF_FLAG_THEN(SEGMENT) MULTILINE(if constexpr (FLAG){SEGMENT})

constexpr double len(int a, int b) { return sqrt(a * a + b * b); }

void PathPlanner::local_planning_map::prepare(const astate& _target,
                                              double (*_safe_map)[MAX_COL],
                                              bool _backward_enabled) {
  target           = _target;
  safe_map         = _safe_map;
  backward_enabled = _backward_enabled;

  time_t time_1 = getTimeStamp();
  calculate_xya_distance_map();
  time_t time_2 = getTimeStamp();
  log_1("xya_distance_map calculated in ", (time_2 - time_1) / 1000, " ms");
}

bool PathPlanner::local_planning_map::is_crashed(int x, int y) const {
  return safe_map[x][y] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION;
}

bool PathPlanner::local_planning_map::is_crashed(const astate& state) const {
  return collision(state.x, state.y, state.a, safe_map, 0.0);
}

bool PathPlanner::local_planning_map::is_crashed(primitive& prim) const {
  const astate& start_state = prim.get_start_state();
  int           ssr         = round(start_state.x);
  int           ssc         = round(start_state.y);
  double        sssafe      = get_maximum_safe_distance(ssr, ssc);
  if (is_in_map(ssr, ssc) == false || prim.get_length() >= sssafe) {
    double safe_s = start_state.s + sssafe;
    for (const auto& state : prim.get_states()) {
      if (state.s <= safe_s) continue;
      int r = round(state.x), c = round(state.y);
      if (is_in_map(r, c)) {
        if (is_crashed(state))
          return true;
        else
          safe_s = state.s + get_maximum_safe_distance(r, c);
      }
    }
  }
  return false;
}

double PathPlanner::local_planning_map::get_heuristic(const astate& state,
                                                      bool can_reverse) const {
  const int x_idx        = (int)round(state.x);
  const int y_idx        = (int)round(state.y);
  const int xya_idx_x    = x_idx >> XYA_MAP_SHIFT_FACTOR;
  const int xya_idx_y    = y_idx >> XYA_MAP_SHIFT_FACTOR;
  const int xya_idx_a    = get_angle_index(state.a);
  double    xya_distance = xya_distance_map[xya_idx_x][xya_idx_y][xya_idx_a];
  double    euclidean_distance =
      euclideanDistance(state.x, state.y, target.x, target.y);
  return max(xya_distance, euclidean_distance);
}

int PathPlanner::local_planning_map::try_get_target_index(
    primitive& primitive) const {
  const auto& start_state = primitive.get_start_state();
  double      dis =
      max(fabs(start_state.x - target.x), fabs(start_state.y - target.y));

  if (dis <= primitive.get_length()) {
    const auto& states = primitive.get_states();
    for (int i = 0, size = states.size(); i < size; ++i)
      if (is_target(states[i])) return i;
  }

  return -1;
}

bool PathPlanner::local_planning_map::is_in_map(const astate& state) const {
  return is_in_map((int)round(state.x), (int)round(state.y));
}

bool PathPlanner::local_planning_map::is_in_map(int row_idx,
                                                int col_idx) const {
  return row_idx >= 0 && row_idx < MAX_ROW && col_idx >= 0 && col_idx < MAX_COL;
}

void PathPlanner::local_planning_map::merge_xya_distance_map(
    pair<double, double> (*output_map)[MAX_COL]) const {
  for (int i = 0; i < MAX_ROW; ++i)
    for (int j = 0; j < MAX_COL; ++j) {
      output_map[i][j].first = 1e9;
      const double* a_vec    = xya_distance_map[i >> XYA_MAP_SHIFT_FACTOR]
                                            [j >> XYA_MAP_SHIFT_FACTOR];
      for (int a = 0; a < XYA_MAP_DEPTH; ++a) {
        if (a_vec[a] < output_map[i][j].first) {
          output_map[i][j].first  = a_vec[a];
          output_map[i][j].second = M_PI - (a + 0.5) * XYA_MAP_DELTA_A;
        }
      }
    }
}

bool PathPlanner::local_planning_map::is_target(const astate& state) const {
  constexpr double dx = 5;
  constexpr double dy = 5;
  constexpr double da = 15 / 180.0 * M_PI;
  if (fabs(state.x - target.x) > dx) return false;
  if (fabs(state.y - target.y) > dy) return false;
  double dangle = wrap_angle_0_2_PI(state.a - target.a);
  dangle        = min(dangle, 2 * M_PI - dangle);
  return dangle <= da;
}

int PathPlanner::local_planning_map::get_angle_index(double ang) {
  return (int)(wrap_angle_0_2_PI(M_PI - ang) / XYA_MAP_DELTA_A) % XYA_MAP_DEPTH;
}

void PathPlanner::local_planning_map::calculate_xya_distance_map() {
  // calculate xya_safe_map by counting safe and unsafe small grid
  // cells in each xya big grid cell. if safe cells are more then
  // the big grid is considered safe.
  memset(xya_safe_map, 0, sizeof(xya_safe_map));
  for (int i = 0; i < XYA_MAP_ROWS - 1; ++i)
    for (int j = 0; j < XYA_MAP_COLS - 1; ++j) {
      int           idx         = (i << XYA_MAP_SHIFT_FACTOR);
      int           jdx         = (j << XYA_MAP_SHIFT_FACTOR);
      int           safe_factor = 0;
      constexpr int xya_cell    = (1 << XYA_MAP_SHIFT_FACTOR);
      int           max_idx     = idx + xya_cell;
      int           max_jdx     = jdx + xya_cell;
      // if the current cell exceed the map
      if (max_idx >= MAX_ROW || max_jdx >= MAX_COL) continue;
      for (; idx < max_idx; ++idx)
        for (; jdx < max_jdx; ++jdx)
          xya_safe_map[i][j] |= (!is_crashed(idx, jdx));
    }

#define vec(a, b)                                         \
  {                                                       \
    a *(XYA_MAP_COLS * XYA_MAP_DEPTH) + b *XYA_MAP_DEPTH, \
        (1 << XYA_MAP_SHIFT_FACTOR) * len(a, b)           \
  }
  constexpr pair<int, double> deltas[] = {vec(1, 0),   vec(1, -1), vec(0, -1),
                                          vec(-1, -1), vec(-1, 0), vec(-1, 1),
                                          vec(0, 1),   vec(1, 1)};
#undef vec
  constexpr int DELTAS_LENGTH    = sizeof(deltas) / sizeof(pair<int, double>);
  constexpr int DEPTHS_PER_DELTA = XYA_MAP_DEPTH / DELTAS_LENGTH;
// https://i.loli.net/2021/03/14/TKVI3kL6mtXcONv.png
#define get_delta_idx(a_idx) \
  (a_idx + (DEPTHS_PER_DELTA / 2)) % XYA_MAP_DEPTH / DEPTHS_PER_DELTA

  // we embedded the x and y value as one integer
  // and add it with deltas[i].first to do the translation
  // this is far more faster than using x and y separately
  double* flatten_map      = (double*)xya_distance_map;
  bool*   flatten_safe_map = (bool*)xya_safe_map;
  ring_buffer_xya.clear();
  memset(is_in_buffer_xya, 0, sizeof(is_in_buffer_xya));
  memset(xya_distance_map, 0x7f, sizeof(xya_distance_map));

  int target_x_idx = (int)round(target.x) >> XYA_MAP_SHIFT_FACTOR;
  int target_y_idx = (int)round(target.y) >> XYA_MAP_SHIFT_FACTOR;
  int target_a_idx = get_angle_index(target.a);
  int target_xya   = target_x_idx * (XYA_MAP_COLS * XYA_MAP_DEPTH) +
                   target_y_idx * XYA_MAP_DEPTH + target_a_idx;

  flatten_map[target_xya] = 0.0;
  ring_buffer_xya.push_back(target_xya);
  is_in_buffer_xya[target_xya] = true;

  BINARY_BRANCH(backward_enabled, {
    while (!ring_buffer_xya.empty()) {
      int xya = ring_buffer_xya.front();
      ring_buffer_xya.pop_front();
      is_in_buffer_xya[xya] = false;
      int    a              = xya % XYA_MAP_DEPTH;
      int    xy0            = xya - a;
      int    delta_idx      = get_delta_idx(a);
      int    zw0_forward    = xy0 + deltas[delta_idx].first;
      double new_dis        = flatten_map[xya] + deltas[delta_idx].second;
#define UPDATE(_zw0_)                                                      \
  MULTILINE(if (_zw0_ >= 0 && flatten_safe_map[_zw0_ / XYA_MAP_DEPTH]) {   \
    for (int da = -1; da <= 1; ++da) {                                     \
      double turning_punishment =                                          \
          deltas[delta_idx].second * fabs(da) * TURNING_PUNISHMENT_FACTOR; \
      int s   = (a + da + XYA_MAP_DEPTH) % XYA_MAP_DEPTH;                  \
      int zws = _zw0_ + s;                                                 \
      if (flatten_map[zws] > new_dis + turning_punishment) {               \
        flatten_map[zws] = new_dis + turning_punishment;                   \
        if (!is_in_buffer_xya[zws]) {                                      \
          ring_buffer_xya.push_back(zws);                                  \
          is_in_buffer_xya[zws] = true;                                    \
        }                                                                  \
      }                                                                    \
    }                                                                      \
  })
      UPDATE(zw0_forward);
      IF_FLAG_THEN({
        int zw0_backward = xy0 - deltas[delta_idx].first;
        new_dis          = flatten_map[xya] +
                  deltas[delta_idx].second * BACKWARD_PUNISHMENT_FACTOR;
        UPDATE(zw0_backward);
      });
    }
  });

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

double PathPlanner::local_planning_map::get_maximum_safe_distance(
    int row_idx, int col_idx) const {
  return max(safe_map[row_idx][col_idx] * M_SQRT1_2 - M_SQRT2 -
                 COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION,
             0.0);
}

double
PathPlanner::local_planning_map::get_minimum_distance_from_map_boundaries(
    const astate& state) const {
  constexpr double row_2 = (MAX_ROW / 2.0);
  constexpr double col_2 = (MAX_COL / 2.0);
  return min(row_2 - fabs(state.x - row_2), col_2 - fabs(state.y - col_2));
}
}  // namespace TiEV