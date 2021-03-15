#include "path_planner.h"
#include "collision_check.h"
#include "look_up_tables/dubins_table/dubins.h"
#include "look_up_tables/reeds_shepp_table/reeds_shepp.h"
#include "splines/Splines.h"
#include "tiev_utils.h"
#include <cmath>
#include <cstring>
#include "log.h"

using namespace std;

namespace TiEV {

PathPlanner::PathPlanner() :
    config(Config::getInstance()),
    view_controller(MessageManager::getInstance()) {}

void PathPlanner::runPlanner(
    const DynamicObjList& dynamic_objs,
    double max_speed,
    bool reverse,
    double abs_safe_map[MAX_ROW][MAX_COL],
    double lane_safe_map[MAX_ROW][MAX_COL],
    const vector<Pose>& start_maintained_path,
    const vector<Pose>& targets,
    double current_speed,
    vector<SpeedPath>& results) {
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
    for(auto& p : this->start_maintained_path) {
        p.v = inf;
    }
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

void PathPlanner::getCostMap(int (*output_map)[MAX_COL]) const {
    if(is_planning) return;
    memset(output_map, 0, sizeof(output_map));
    for (int i = 0; i < targets.size(); ++i)
        hybrid_astar_planners[i].merge_history_map(output_map);
}

void PathPlanner::getDistanceMaps(
    double (*xy_dis_map)[MAX_COL],
    pair<double, double> (*xya_dis_map)[MAX_COL]) const {
    if(is_planning) return;
    memset(xy_dis_map, 0x7f, sizeof(double) * MAX_ROW * MAX_COL);
    memset(xya_dis_map, 0x7f, sizeof(double) * MAX_ROW * MAX_COL);
    for (int i = 0; i < targets.size(); ++i) {
        hybrid_astar_planners[i].merge_xy_distance_map(xy_dis_map);
        hybrid_astar_planners[i].merge_xya_distance_map(xya_dis_map);
    }
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
    stop_s = (speed * speed) / (2 * MIU * GRAVITY) / GRID_RESOLUTION * 1.1;
}

void PathPlanner::plan() {
    if(is_planning) return;
    is_planning = true;

    log(0, "planning started");

    for(int i = 0; i < MAX_TARGET_NUM; i++) {
        speed_paths[i].path.clear();
        have_result[i] = false;
    }

    log(0, "planning result cleared");

    if(start_point.backward) {
        backward_enabled = true;
        log(0, "start point is backward, force enable backward");
    }

    // merge united safe map
    for(int i = 0; i < MAX_ROW; ++i)
        for(int j = 0; j < MAX_COL; ++j)
            safe_map[i][j] = min(
                abs_safe_map[i][j],
                lane_safe_map[i][j]);
    log(0, "safe map merged");

    vector<thread> tasks(targets.size());
    // start planner_thread
    for(int i = 0; i < targets.size(); ++i) {
        thread t(&PathPlanner::planner_thread, this, i);
        tasks[i] = std::move(t);
    }
    log(0, targets.size(), " thread(s) created");

    // wait for plans
    for(int i = 0; i < tasks.size(); ++i) tasks[i].join();

    log(0, "sending data to visualization\n");
    view_controller->setTargets(targets);
    view_controller->setStartPoint(start_point);
    view_controller->setSafeMap(safe_map);
    view_controller->clearPaths();
    for(int i = 0; i < tasks.size(); ++i)
        if(have_result[i])
            view_controller->setPath(speed_paths[i].path);

    is_planning = false;
    log(0, "path planner done");
}

void PathPlanner::planner_thread(int target_index) {
    log(0, "thread ", target_index, " started");
    astate start_state = {
        start_point.x,
        start_point.y,
        start_point.ang,
        0.0,    // s
        start_point.k,
        start_point.v < 0.0
    }, target_state = {
        targets[target_index].x,
        targets[target_index].y,
        targets[target_index].ang,
        0.0,    // s
        targets[target_index].k,
        targets[target_index].v < 0.0
    };

    hybrid_astar_planner* planner =
        hybrid_astar_planners + target_index;

    log(0, "thread ", target_index, " planner ready");
    planner->plan(start_state, target_state,
        safe_map, config->plan_time_limit_ms * 1000);
    log(0, "thread ", target_index, " planned");

    if (planner->get_have_result()) {
        log(0, "thread ", target_index, " have result = true");
        const auto& result = planner->get_result();
        speed_paths[target_index].path.clear();
        speed_paths[target_index].path.reserve(result.size());
        for (const auto& bstate : result) {
            Pose    p;
            p.x   = bstate.x;
            p.y   = bstate.y;
            p.ang = bstate.a;
            p.updateGlobalCoordinate(start_point);
            p.s = bstate.s * GRID_RESOLUTION;
            p.k = bstate.curvature / GRID_RESOLUTION;
            p.backward = bstate.is_backward;
            speed_paths[target_index].path.push_back(p);
        }
        log(0, "thread ", target_index, " speed path generated, length = ",
            speed_paths[target_index].path.size());
        have_result[target_index] = true;
        planSpeed(target_index);
    }
    else log(0, "thread ", target_index, " no result, abort");
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
    for(auto& p : result_tail) {
        p.v = 0;
        p.t = inf;
    }

    if(end_point != speed_paths[target_index].path.size()) {
        speed_paths[target_index].path.resize(end_point);
        speed_paths[target_index].path.back().v = 0;
    }

    speed_limits[target_index].clear();
    speed_limits[target_index].reserve(speed_paths[target_index].path.size());

    // conversion
    for(auto& point : speed_paths[target_index].path) {
        double max_speed = point.v;
        if(point.backward) {
            max_speed = 2;
            point.ang = PI + point.ang;
        }
        speed_limits[target_index].emplace_back(point.s, min(sqrt(MIU * GRAVITY / (point.k + 0.0001)) * 5.5, max_speed));
    }

    // speed_limits[target_index][0].second = current_speed;
    speed_paths[target_index].path.front().v = current_speed;
    speed_paths[target_index] =
        SpeedOptimizer::RunSpeedOptimizer(dynamic_obj_list.dynamic_obj_list, speed_paths[target_index].path, speed_limits[target_index], speed_paths[target_index].path.back().s);

    // anti-conversion
    for(auto& point : speed_paths[target_index].path) {
        if(point.backward) {
            point.ang = point.ang - PI;
            point.v   = -point.v;
            point.a   = -point.a;
        }
    }

    speed_paths[target_index].path.insert(speed_paths[target_index].path.end(), result_tail.begin(), result_tail.end());
#endif
}
}  // namespace TiEV
