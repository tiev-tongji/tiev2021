#include "path_planner.h"
#include "collision_check.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "look_up_tables/reeds_shepp_table/reeds_shepp.h"
#include "splines/Splines.h"
#include "tiev_utils.h"
#include <cmath>
#include <cstring>

using namespace std;

namespace TiEV {

const double inf = 1e10;

const double sqrt2 = 1.414213562;
const double sqrt5 = 2.2360679775;
const int    dx[]  = { -2, -1, 0, 1, 2 };
const int    dy[]  = { -2, -1, 0, 1, 2 };
const int    ldx   = sizeof(dx) / sizeof(dx[0]);
const int    ldy   = sizeof(dy) / sizeof(dy[0]);

const double g_tims_miu = GRAVITY * MIU;

const double max_deceleration = -2;
const double max_acceleration = 3;

const double dd[5][5] = { 2.0 * sqrt2, sqrt5, 2.0,   sqrt5, 2.0 * sqrt2, sqrt5, sqrt2, 1.0,         sqrt2, sqrt5, 2.0,   1.0,        0.0,
                          1.0,         2.0,   sqrt5, sqrt2, 1.0,         sqrt2, sqrt5, 2.0 * sqrt2, sqrt5, 2.0,   sqrt5, 2.0 * sqrt2 };
const double lengths[]        = { 10.0 / GRID_RESOLUTION, 5.0 / GRID_RESOLUTION, 2.0 / GRID_RESOLUTION };
const int    length_num       = sizeof(lengths) / sizeof(lengths[0]);
const double radiuses[]       = { 5.0 / GRID_RESOLUTION, 14.0 / GRID_RESOLUTION, 20.0 / GRID_RESOLUTION };
const double circle_lengths[] = { lengths[length_num - 1], lengths[length_num - 1], lengths[length_num - 1] };
const int    radius_num       = sizeof(radiuses) / sizeof(radiuses[0]);

PathPlanner::PathPlanner()
    : config(Config::getInstance()), view_controller(MessageManager::getInstance()), astar_dubins_distance_table(config->dubins_distance_table_path),
      astar_rs_distance_table(config->rs_distance_table_path) {

    aStarInitializeExtendedPositions();
}

void PathPlanner::runPlanner(const DynamicObjList& dynamic_objs, double max_speed, bool reverse, double abs_safe_map[MAX_ROW][MAX_COL], double lane_safe_map[MAX_ROW][MAX_COL],
                             const vector<Pose>& start_maintained_path, const vector<Pose>& targets, double current_speed, vector<SpeedPath>& results) {
    setAbsSafeMap(abs_safe_map);
    setLaneSafeMap(lane_safe_map);
    setStartMaintainedPath(start_maintained_path);
    setTargets(targets);
    setBackwardEnabled(reverse);
    setCurrentSpeed(current_speed);
    setDynamicObjList(dynamic_objs);
    setVelocityLimit(max_speed);
    plan();
    getResults(results);
    MessageManager*   msg_m = MessageManager::getInstance();
    visVISUALIZATION& vis   = msg_m->visualization;
    memcpy(vis.used_map, used_map, sizeof(vis.used_map));
    vis.paths_size = results.size();
    for(const auto& speed_path : results) {
        visPath vis_path;
        vis_path.path_size = speed_path.path.size();
        for(const auto& p : speed_path.path) {
            visPoint vp;
            vp.x = p.x;
            vp.y = p.y;
            vis_path.path.push_back(vp);
        }
        vis.paths.push_back(vis_path);
    }
}

void PathPlanner::setStartMaintainedPath(const vector<Pose>& start_maintained_path) {
    if(is_planning) return;
    this->start_maintained_path.clear();
    this->start_maintained_path = start_maintained_path;
    for(auto& p : this->start_maintained_path)
        p.v                                        = inf;
    if(!start_maintained_path.empty()) start_point = start_maintained_path.back();
    if(!start_point.in_map() || start_maintained_path.empty()) {
        start_point = Pose(CAR_CEN_ROW, CAR_CEN_COL, PI);
    }
}

void PathPlanner::setTargets(const vector<Pose>& targets) {
    if(is_planning) return;
    this->targets = targets;
    if(targets.size() > MAX_TARGET_NUM) this->targets.resize(MAX_TARGET_NUM);
}

bool PathPlanner::getResults(vector<SpeedPath>& results) {
    if(is_planning) return false;
    results.clear();
    bool any_result = false;
    for(int i = 0; i < MAX_TARGET_NUM; ++i)
        any_result |= have_result[i];
    if(any_result) {
        for(int i = 0; i < MAX_TARGET_NUM; ++i)
            if(have_result[i]) results.emplace_back(speed_paths[i]);
    }
    return any_result;
}

void PathPlanner::setBackwardEnabled(bool enabled) {
    if(is_planning) return;
    backward_enabled = enabled;
}

void PathPlanner::setVelocityLimit(double speed) {
    if(is_planning) return;
    velocity_limit = speed;
}

void PathPlanner::setAbsSafeMap(double map[MAX_ROW][MAX_COL]) {
    if(is_planning) return;
    memcpy(this->abs_safe_map, map, sizeof(this->abs_safe_map));
}

void PathPlanner::setLaneSafeMap(double map[MAX_ROW][MAX_COL]) {
    if(is_planning) return;
    memcpy(this->lane_safe_map, map, sizeof(this->lane_safe_map));
}

void PathPlanner::setSafeMap(double map[MAX_ROW][MAX_COL]) {
    cerr << "PathPlanner::setSafeMap -> deleted function!" << endl;
    throw 0;
    if(is_planning) return;
    memcpy(this->abs_safe_map, map, sizeof(this->safe_map));
}

void PathPlanner::setDynamicObjList(const DynamicObjList& dynamic_obj_list) {
    if(!dynamic_obj_list.detected) {
        this->dynamic_obj_list.detected = false;
        this->dynamic_obj_list.dynamic_obj_list.clear();
    }
    else {
        this->dynamic_obj_list = dynamic_obj_list;
    }
    // MAYBE: coordinate conversion
}

void PathPlanner::setCurrentSpeed(double speed) {
    if(is_planning) return;
    current_speed = speed;
    for(int i = 0; i < (sizeof(speeds) / sizeof(double)); ++i)
        if(speeds[i] > current_speed) {
            current_speed_id = (i == 0 ? 0 : (i - 1));
            break;
        }
    stop_s = (speed * speed) / (2 * MIU * GRAVITY) / GRID_RESOLUTION * 1.1;
}

bool PathPlanner::timeout() {
#ifndef NO_TIME_LIMIT
    static int k = 0;
    if(k++ < 1024) return false;
    k = 0;
    return getTimeStamp() >= plan_start_time + config->plan_time_limit_ms * 1000;
#else
    return false;
#endif
}

void PathPlanner::plan() {
    for(int i = 0; i < MAX_TARGET_NUM; i++) {
        speed_paths[i].path.clear();
        have_result[i] = false;
    }
    if(is_planning) return;

    is_planning = true;
    memset(have_result, 0, sizeof(have_result));

    if(start_point.backward) backward_enabled = true;
    view_controller->setTargets(targets);

    // Pretreatments for each plan method

    aStarPretreatment();

    // Start plan
    vector<thread> tasks(targets.size());
    plan_start_time = getTimeStamp();
    for(int i = 0; i < targets.size(); ++i) {
        thread t(&PathPlanner::run, this, i);
        tasks[i] = std::move(t);
    }

    for(int i = 0; i < tasks.size(); ++i)
        tasks[i].join();

    view_controller->setStartPoint(start_point);
    view_controller->setSafeMap(safe_map);

    view_controller->clearPaths();
    for(int i = 0; i < tasks.size(); ++i)
        if(have_result[i]) view_controller->setPath(speed_paths[i].path);

    memset(used_map, 0, sizeof(used_map));
    memcpy(used_map, astar_used[0].map, sizeof(used_map));

    view_controller->setUsedMap(used_map);

    is_planning = false;
}

void PathPlanner::run(int target_index) {
    aStarPlan(target_index);
    if(!have_result[target_index]) return;
    vector<Pose>& res = speed_paths[target_index].path;

    if(start_maintained_path.size() > 1) {
        double offset = start_maintained_path.back().s;
        res.insert(res.begin(), start_maintained_path.begin(), start_maintained_path.end() - 1);
        for(int i = start_maintained_path.size() - 1; i < res.size(); ++i)
            res[i].s += offset;
    }

    for(auto& p : res)
        p.v = velocity_limit;

    res.back().v    = targets[target_index].v;
    res[0].backward = res[1].backward;

#ifndef NO_SPEED_PLANNER
    planSpeed(target_index);
#endif
}

void PathPlanner::planSpeed(int target_index) {
#ifndef NO_SPEED_PLANNER
    int end_point = speed_paths[target_index].path.size();
    for(int i = 1; i < speed_paths[target_index].path.size(); ++i)
        if(speed_paths[target_index].path[i].backward != speed_paths[target_index].path[i - 1].backward) {
            end_point = i;
            break;
        }

    vector<Pose> result_tail;
    result_tail.insert(result_tail.begin(), speed_paths[target_index].path.begin() + end_point, speed_paths[target_index].path.end());
    for(auto& p : result_tail)
        p.v = 0;

    if(end_point != speed_paths[target_index].path.size()) {
        speed_paths[target_index].path.resize(end_point);
        speed_paths[target_index].path.back().v = 0;
    }

    speed_limits[target_index].clear();
    speed_limits[target_index].reserve(speed_paths[target_index].path.size());

    // conversion
    for(auto& point : speed_paths[target_index].path) {
        if(!point.backward) point.ang = PI - point.ang;
        speed_limits[target_index].emplace_back(point.s, min(sqrt(g_tims_miu / (point.k + 0.0001)), point.v) * 0.7);
    }

    // speed_limits[target_index][0].second = current_speed;
    speed_paths[target_index].path.front().v = current_speed;
    speed_paths[target_index] =
        SpeedOptimizer::RunSpeedOptimizer(dynamic_obj_list.dynamic_obj_list, speed_paths[target_index].path, speed_limits[target_index], speed_paths[target_index].path.back().s);

    // anti-conversion
    for(auto& point : speed_paths[target_index].path) {
        if(!point.backward)
            point.ang = PI - point.ang;
        else {
            point.v = -point.v;
            point.a = -point.a;
        }
    }

    speed_paths[target_index].path.insert(speed_paths[target_index].path.end(), result_tail.begin(), result_tail.end());
#endif
}

// AStar

void PathPlanner::aStarPretreatment() {
    for(int i = 0; i < MAX_ROW; ++i)
        for(int j          = 0; j < MAX_COL; ++j)
            safe_map[i][j] = min(abs_safe_map[i][j], lane_safe_map[i][j]);

    for(int i = 0; i < targets.size(); ++i)
        calculateDistanceBySPFA(safe_map, astar_distance_to_multi_targets[i].map, make_pair(targets[i].x, targets[i].y), astar_queues[i]);

    memset(astar_lowest_cost_state, 0xff, sizeof(astar_lowest_cost_state));
    // This will get a very big double
    memset(astar_used, 0, sizeof(astar_used));

    for(int i = 0; i < MAX_TARGET_NUM; ++i) {
        astar_stored_states[i].clear();
        if(!astar_priority_queues[i].empty()) {
            priority_queue<pair<double, int>> empty_que;
            swap(empty_que, astar_priority_queues[i]);
        }
    }
}

double PathPlanner::aStarHeuristic(int target_index, const astate& current) {
#ifndef NO_HEURISTIC
    double tx = targets[target_index].x, ty = targets[target_index].y;
    double ta   = targets[target_index].ang;
    double q0[] = { current.x, current.y, current.a }, q1[] = { tx, ty, ta };
    double curve_distance;
    if(backward_enabled)
        curve_distance = astar_rs_distance_table.getDistance(q0, q1);
    else
        curve_distance = astar_dubins_distance_table.getDistance(q0, q1);
    // double euclidean                                      = euclideanDistance(current.x, current.y, tx, ty);
    // double p1                                             = max(euclidean, curve_distance);
    double spfa_dis                                       = aStarGetDistanceToTarget(target_index, current.x, current.y);
    double curvature_changed_punish                       = 0;
    if(current.prior_index >= 0) curvature_changed_punish = 200 * fabs(current.curvature - astar_stored_states[target_index][current.prior_index].curvature);
    return (max(curve_distance, spfa_dis) + curvature_changed_punish);
#else
    return 0;
#endif
}

bool PathPlanner::aStarIsTarget(int target_index, const astate& current) {
    const double ts = 2.0;
    const double ta = 0.3;
    if(abs(current.x - targets[target_index].x) > ts) return false;
    if(abs(current.y - targets[target_index].y) > ts) return false;
    if(abs(current.a - targets[target_index].ang > ta)) return false;
    return true;
}

void PathPlanner::aStarPlan(int target_index) {
    vector<astate>& stored_states = astar_stored_states[target_index];
    priority_queue<pair<double, int>>& using_queue = astar_priority_queues[target_index];

    astate start_state(start_point.x, start_point.y, start_point.ang);

    start_state.cost        = 0;
    start_state.prior_index = -1;
    stored_states.push_back(start_state);
    using_queue.push(make_pair(-aStarHeuristic(target_index, start_state), 0));
    aStarUpdateCost(0, target_index);

    vector<astate>         analytic_expansion_states;
    vector<vector<astate>> extended_states;

    astate current_state(0, 0, 0);                  // Current most valuable state
    int    end_index = -1, state_cnt = -1;          // if end_index >= 0 then target reached
    bool   analytic_expansion_first_tried = false;  // Try it once it can at the beginning

    while(!timeout() && !using_queue.empty()) {
        int current_index = using_queue.top().second;
        using_queue.pop();
        current_state            = stored_states[current_index];
        double current_euclidean = euclideanDistance(current_state.x, current_state.y, targets[target_index].x, targets[target_index].y);
        double current_spfa      = aStarGetDistanceToTarget(target_index, current_state.x, current_state.y);
        int    N                 = aStarAnalyticExpansionsInterval(current_spfa);

        ++state_cnt;
#ifndef NO_RS
        if((!backward_enabled || current_state.cost >= stop_s - 0.5) && (state_cnt >= N || !analytic_expansion_first_tried)) {

            analytic_expansion_first_tried = true;
            state_cnt                      = 0;

            double r = max(1.0 / curvatures[current_speed_id] / GRID_RESOLUTION, 5.0 / GRID_RESOLUTION);

            if(aStarAnalyticExpansion(target_index, current_state, analytic_expansion_states, r)) {
                if(analytic_expansion_states.size() >= 0) {
                    analytic_expansion_states[0].prior_index = current_index;
                    for(int i                                    = 1; i < analytic_expansion_states.size(); ++i)
                        analytic_expansion_states[i].prior_index = stored_states.size() + i - 1;
                    stored_states.insert(stored_states.end(), analytic_expansion_states.begin(), analytic_expansion_states.end());
                    end_index = stored_states.size() - 1;
                    break;
                }
            }
            else if(analytic_expansion_states.size()) {
                // If the analytic expansion is failed, select some points and add them to open-list.
                int jump_step                            = (int)ceil(5.0 / config->a_star_extention_step_meter);
                int jump_size                            = (analytic_expansion_states.size() / 2 / jump_step) * jump_step;
                int old_l                                = stored_states.size();
                analytic_expansion_states[0].prior_index = current_index;
                stored_states.insert(stored_states.end(), analytic_expansion_states.begin(), analytic_expansion_states.begin() + jump_size);
                for(int i = old_l + 1; i < old_l + jump_size; ++i) {
                    stored_states[i].prior_index = i - 1;
                    if((i - old_l) % jump_step == 0) {
                        using_queue.push(make_pair(-aStarHeuristic(target_index, stored_states[i]) - stored_states[i].cost, i));
                    }
                }
            }
        }
#endif

        aStarExtend(current_state, extended_states, current_euclidean);

        for(auto& path : extended_states) {
            if(path.size() <= 1) continue;
            bool could_reach_target = (current_euclidean <= (path.back().cost - path.front().cost) * 2);
            path[1].prior_index     = current_index;
            int old_l               = stored_states.size() + 1;
            stored_states.insert(stored_states.end(), path.begin() + 1, path.end());
            for(int i                        = old_l; i < stored_states.size(); ++i)
                stored_states[i].prior_index = i - 1;

            if(could_reach_target) {
                for(int i = old_l; i < stored_states.size(); ++i)
                    if(aStarIsTarget(target_index, stored_states[i])) {
                        end_index = i;
                        break;
                    }
            }

            if(end_index >= 0) break;
            if(aStarUpdateCost(stored_states.size() - 1, target_index))
                using_queue.push(make_pair(-aStarHeuristic(target_index, stored_states.back()) - stored_states.back().cost, stored_states.size() - 1));
        }

        if(end_index >= 0) break;
    }

    double end = getTimeStamp();
    cout << "Target" << target_index << " planning time" << (end - plan_start_time) / 1000 << "ms" << endl;

#ifdef COUT_DEBUG_INFO
    cout << stored_states.size() << " iterations" << endl;
#endif

#ifndef SHOW_ALL_STATES
    // collect the path
    if(end_index > 0) {
        speed_paths[target_index].path.clear();
        int             cur = end_index, count = 0;
        vector<astate>  tmp_states;
        vector<astate>* last_trans_prim = NULL;
        for(int tmp = cur; tmp >= 0; tmp = stored_states[tmp].prior_index) {
            astate& state = stored_states[tmp];
            tmp_states.clear();
            if(last_trans_prim)
                aStarTransformPrimitive(state, last_trans_prim, tmp_states);
            else
                tmp_states.push_back(state);
            last_trans_prim = state.trans_prim;
            for(int i = tmp_states.size() - 1; i >= 0; --i) {
                astate& st = tmp_states[i];
                Pose    p;
                p.x   = st.x;
                p.y   = st.y;
                p.ang = st.a;
                p.updateGlobalCoordinate(start_point);
                p.s        = st.cost * GRID_RESOLUTION;
                p.k        = fabs(st.curvature / GRID_RESOLUTION);
                p.backward = st.backward;
                speed_paths[target_index].path.push_back(p);
#ifdef COUT_DEBUG_INFO
                cerr << st.x << ", " << st.y << ", " << st.a << "," << endl;
#endif
            }
        }
        std::reverse(speed_paths[target_index].path.begin(), speed_paths[target_index].path.end());
        speed_paths[target_index].path[0].backward = speed_paths[target_index].path[1].backward;
        have_result[target_index]                  = true;
#ifdef COUT_DEBUG_INFO
        cout << "total length: " << speed_paths[target_index].path.back().s << endl;
#endif
    }
    else {
        speed_paths[target_index].path.clear();
        have_result[target_index] = false;
    }
#endif

    for(auto& p : stored_states)
        astar_used[target_index].map[(int)round(p.x)][(int)round(p.y)] = true;
}

int PathPlanner::aStarAnalyticExpansionsInterval(double distance) {
    return min(config->a_star_analytic_expansion_max_N, (int)round(exp((distance - config->a_star_analytic_expansion_param_t) / config->a_star_analytic_expansion_param_k)));
}

bool PathPlanner::aStarAnalyticExpansion(int target_index, const astate& state, vector<astate>& expansion_states, double radius) {
    expansion_states.clear();
    double       q0[] = { state.x, state.y, state.a };
    double       q1[] = { targets[target_index].x, targets[target_index].y, targets[target_index].ang };
    const double step = config->a_star_extention_step_meter / GRID_RESOLUTION;
    double       q[3], last_q[3] = { state.x, state.y, state.a }, x = step, length;
    if(!backward_enabled) {
        DubinsPath tmpath;
        dubins_shortest_path(&tmpath, q0, q1, radius);
        length = dubins_path_length(&tmpath);
        while(x < length) {
            dubins_path_sample(&tmpath, x, q);
            Pose   p(q[0], q[1], q[2]);
            double expansion_r = current_speed * 0.06;
            if(!collision(p, abs_safe_map, expansion_r) && !collision(p, lane_safe_map)) {
                // if(isCarSafeHere(q[0], q[1], PI - q[2], abs_safe_map, lane_safe_map, current_speed)) {
                expansion_states.emplace_back(q[0], q[1], q[2]);
                expansion_states.back().curvature = ((q[2] == last_q[2]) ? 0 : (1.0 / radius));
                expansion_states.back().cost      = state.cost + x;
                expansion_states.back().backward  = (((q[0] - last_q[0]) * cos(q[2]) + (q[1] - last_q[1]) * sin(q[2])) < 0);
                memcpy(last_q, q, 3 * sizeof(double));
            }
            else
                return false;
            x += step;
        }
    }
    else {
        ReedsSheppStateSpace                 rs(radius);
        ReedsSheppStateSpace::ReedsSheppPath path = rs.reedsShepp(q0, q1);
        length                                    = path.length() * rs.rho_;
        while(x < length) {
            rs.interpolate(q0, path, x / rs.rho_, q);
            Pose   p(q[0], q[1], q[2]);
            double expansion_r = current_speed * 0.06;
            if(!collision(p, abs_safe_map, expansion_r) && !collision(p, lane_safe_map)) {
                // if(isCarSafeHere(q[0], q[1], PI - q[2], abs_safe_map, lane_safe_map, current_speed)) {
                expansion_states.emplace_back(q[0], q[1], q[2]);
                expansion_states.back().curvature = ((q[2] == last_q[2]) ? 0 : (1.0 / radius));
                expansion_states.back().cost      = state.cost + x;
                expansion_states.back().backward  = (((q[0] - last_q[0]) * cos(q[2]) + (q[1] - last_q[1]) * sin(q[2])) < 0);
                memcpy(last_q, q, 3 * sizeof(double));
            }
            else
                return false;
            x += step;
        }
    }

    if(!expansion_states.empty()) expansion_states[0].curvature = state.curvature;
    return !expansion_states.empty();
}

void PathPlanner::aStarExtend(const astate& source, vector<vector<astate>>& destination, double current_euclidean) {
    destination.clear();

    vector<vector<astate>>* primitives_list[] = { &astar_speed_primitives[current_speed_id].forward_primitives, &astar_speed_primitives[current_speed_id].backward_primitives };
    const int               plist_l           = sizeof(primitives_list) / sizeof(void*);

    const double sina = sin(source.a);
    const double cosa = cos(source.a);
    const double ppa  = source.a;
    const double pix2 = PI * 2.0;
    // don't change direction before stop_s
    if(source.cost < stop_s - 0.5) {
        if(start_point.backward)
            primitives_list[0] = NULL;
        else
            primitives_list[1] = NULL;
    }

    // don't extend backward when not enabled
    if(!backward_enabled) primitives_list[1] = NULL;

    //
    int sfar = min(abs_safe_map[(int)round(source.x)][(int)round(source.y)], lane_safe_map[(int)round(source.x)][(int)round(source.y)]);

    for(int primitive_i = 0; primitive_i < plist_l; ++primitive_i) {
        vector<vector<astate>>* primitives_ptr = primitives_list[primitive_i];
        if(!primitives_ptr) continue;
        int start_i = destination.size();
        destination.insert(destination.end(), primitives_ptr->begin(), primitives_ptr->end());
        for(int i = start_i; i < destination.size(); ++i) {
            double arc_length = destination[i].back().cost;
            // if the minimum distance between front_state and any obstacle
            // is smaller than arc_length then this primitive must be safe
            // if(0) {
            const auto& back_p   = destination[i].back();
            double      back_ang = ppa + back_p.a;
            while(back_ang <= -PI)
                back_ang += pix2;
            while(back_ang > PI)
                back_ang -= pix2;
            double back_x      = cosa * back_p.x - sina * back_p.y + source.x;
            double back_y      = sina * back_p.x + cosa * back_p.y + source.y;
            Pose   back_pose   = Pose(back_x, back_y, back_ang);
            double expansion_r = current_speed * 0.06;
            if(sfar > arc_length && current_euclidean > arc_length && !collision(back_pose, lane_safe_map) && !collision(back_pose, abs_safe_map, expansion_r)) {
                auto back_state  = destination[i].back();
                auto front_state = destination[i].front();
                destination[i].resize(2);
                destination[i][0]            = front_state;
                destination[i][1]            = back_state;
                destination[i][1].trans_prim = &(primitives_ptr->operator[](i - start_i));
            }

            for(int j = 0; j < destination[i].size(); ++j) {
                auto& p = destination[i][j];
                p.a     = ppa + p.a;
                // TODO: maybe not essential
                while(p.a <= -PI)
                    p.a += pix2;
                while(p.a > PI)
                    p.a -= pix2;
                double x = cosa * p.x - sina * p.y + source.x;
                double y = sina * p.x + cosa * p.y + source.y;
                p.x      = x;
                p.y      = y;
                p.cost += source.cost;
                Pose   pp(p.x, p.y, p.a);
                double expansion_r = current_speed * 0.06;
                if(collision(pp, abs_safe_map, expansion_r) || collision(pp, lane_safe_map)) {
                    // if(!isCarSafeHere(p.x, p.y, p.a, abs_safe_map, lane_safe_map, current_speed)) {
                    // destination[i].clear();
                    destination[i].resize(j);
                    break;
                }
            }
        }
    }
}

void PathPlanner::aStarTransformPrimitive(const astate& source, vector<astate>* primitive_ptr, vector<astate>& dest) {
    dest              = *primitive_ptr;
    const double sina = sin(source.a);
    const double cosa = cos(source.a);
    const double ppa  = source.a;
    const double pix2 = PI * 2.0;
    for(auto& p : dest) {
        p.a = ppa + p.a;  // TODO: maybe not essential while(p.a < -PI)
        while(p.a <= -PI)
            p.a += pix2;
        while(p.a > PI)
            p.a -= pix2;
        double x = cosa * p.x - sina * p.y + source.x;
        double y = sina * p.x + cosa * p.y + source.y;
        p.x      = x;
        p.y      = y;
        p.cost += source.cost;
    }
}

bool PathPlanner::aStarUpdateCost(int state_index, int target_index) {
    const astate& state     = astar_stored_states[target_index][state_index];
    double        state_ang = state.a;
    while(state_ang >= 2 * PI)
        state_ang -= 2 * PI;
    while(state_ang < 0)
        state_ang += 2 * PI;
    const int angle_id = (int)round((state_ang) / UNIT_DISCRETE_ANGLE);
    const int x = (int)round(state.x), y = (int)round(state.y);
    int       index = astar_lowest_cost_state[target_index].map[x][y][angle_id];
    if(index < 0 || astar_stored_states[target_index][index].cost > state.cost) {
        astar_lowest_cost_state[target_index].map[x][y][angle_id] = state_index;
        return true;
    }
    else
        return false;
}

bool PathPlanner::aStarIsLineSafe(const astate& a, const astate& b) {
    const double step     = 1;
    const double length   = euclideanDistance(a.x, a.y, b.x, b.y);
    const int    step_num = floor(length / 0.5);
    const double dx       = (b.x - a.x) / step_num;
    const double dy       = (b.y - a.y) / step_num;
    astate       tmp(a.x, a.y, b.a);
    for(int i = 0; i <= step_num; ++i) {
        Pose   p(tmp.x, tmp.y, tmp.a);
        double expansion_r = current_speed * 0.06;
        if(collision(p, abs_safe_map, expansion_r) || collision(p, lane_safe_map)) return false;
        // if(!isCarSafeHere(tmp.x, tmp.y, tmp.a, abs_safe_map, lane_safe_map, current_speed)) return false;
        tmp.x += dx;
        tmp.y += dy;
    }

    return true;
}

double PathPlanner::aStarGetDistanceToTarget(int target_index, double x, double y) {
    int    xs[2]   = { (int)floor(x), (int)ceil(x) };
    int    ys[2]   = { (int)floor(y), (int)ceil(y) };
    double min_dis = inf;
    for(int i = 0; i < 2; ++i)
        for(int j = 0; j < 2; ++j) {
            double dis = euclideanDistance(x, y, xs[i], ys[j]) + astar_distance_to_multi_targets[target_index].map[xs[i]][ys[j]];
            min_dis    = min(dis, min_dis);
        }
    return min_dis;
}

void PathPlanner::primitives::addPrimitive(vector<astate>& forward_primitive, bool need_mirror) {
    forward_primitives.push_back(forward_primitive);

    backward_primitives.push_back(forward_primitive);
    for(auto& p : backward_primitives.back()) {
        p.x        = -p.x;
        p.y        = -p.y;
        p.a        = p.a + PI;
        p.backward = true;
    }

    if(need_mirror) {
        forward_primitives.push_back(forward_primitive);
        for(auto& p : forward_primitives.back()) {
            p.y = -p.y;
            p.a = -p.a;
        }

        backward_primitives.push_back(forward_primitive);
        for(auto& p : backward_primitives.back()) {
            p.x = -p.x;
            p.a = PI - p.a;
        }
    }
}

void PathPlanner::aStarInitializeExtendedPositions() {
    vector<astate> arcs[speed_cnt];
    vector<astate> lines[speed_cnt];
    for(int spdid = 0; spdid < speed_cnt; ++spdid) {
        generateLeftArc(spdid, arcs[spdid]);
        generateLine(spdid, lines[spdid]);
    }
    // create speed_primitives
    for(int spdid = 0; spdid < speed_cnt; ++spdid) {
        // create arcs with smaller curvature first
        astar_speed_primitives[spdid].addPrimitive(lines[spdid], false);
        for(int i = speed_cnt - 1; i >= spdid; --i) {
            astar_speed_primitives[spdid].addPrimitive(arcs[i]);
        }
    }
}

void PathPlanner::generateLeftArc(int speed_id, vector<astate>& positions) {
    const double step = config->a_star_extention_step_meter;
    positions.clear();
    double r        = 1.0 / curvatures[speed_id];
    double l        = arc_lengths[speed_id];
    double center_x = 0, center_y = r;
    double theta = step / r, alpha = 0;
    for(double cost = 0; cost <= l; cost += step, alpha += theta) {
        // astate& position = positions[real_count];
        astate position;
        double sina        = sin(alpha);
        double cosa        = cos(alpha);
        position.x         = (r * sina + center_x) / GRID_RESOLUTION;
        position.y         = (-r * cosa + center_y) / GRID_RESOLUTION;
        position.cost      = cost / GRID_RESOLUTION;
        position.a         = alpha;
        position.curvature = 1 / (r / GRID_RESOLUTION);
        positions.push_back(position);
    }
}

void PathPlanner::generateLine(int speed_id, vector<astate>& positions) {
    const double step = config->a_star_extention_step_meter;
    positions.clear();
    double l = arc_lengths[speed_id];
    for(double cost = 0; cost <= l; cost += step) {
        astate pos;
        pos.y        = 0.0;
        pos.a        = 0;
        pos.x        = cost / GRID_RESOLUTION;
        pos.backward = false;
        pos.cost     = cost / GRID_RESOLUTION;
        positions.push_back(pos);
    }
}

// Tool methods
void PathPlanner::calculateDistanceBySPFA(double map[MAX_ROW][MAX_COL], double output[MAX_ROW][MAX_COL], const pair<int, int>& target, queue<pair<int, int>>& using_queue) {

    while(!using_queue.empty())
        using_queue.pop();
    for(int i = 0; i < MAX_ROW; ++i)
        for(int j        = 0; j < MAX_COL; ++j)
            output[i][j] = inf;

    output[target.first][target.second] = 0;
    using_queue.push(target);

    while(!using_queue.empty()) {
        const int x = using_queue.front().first;
        const int y = using_queue.front().second;
        using_queue.pop();
        for(int i = 0; i < ldx; ++i)
            for(int j = 0; j < ldy; ++j) {
                const int tx = x + dx[i], ty = y + dy[j];
                if(tx < 0 || ty < 0 || tx >= MAX_ROW || ty >= MAX_COL || map[tx][ty] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) continue;
                if(output[x][y] + dd[i][j] < output[tx][ty]) {
                    output[tx][ty] = output[x][y] + dd[i][j];
                    using_queue.push(make_pair(tx, ty));
                }
            }
    }
}
}
