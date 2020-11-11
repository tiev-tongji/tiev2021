#ifndef _TIEV_PLANNER_CONFIG_H_
#define _TIEV_PLANNER_CONFIG_H_

#include "pose.h"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace TiEV {

enum ControlMode {
    PlanningWithMapMode,    //规划模式，使用地图RoadMode和SpeedMode
    TrakingWithMapMode,     //循迹模式，使用地图RoadMode和SpeedMode
    PlanningWithDebugMode,  //规划模式，使用debugRoadMode和debugSpeedMode
    TrakingWithDebugMode    //循迹模式，使用debugRoadMode和debugSpeedMode
};

enum ObjectType { CAR = 0, BYCICLE = 1, PEDESTRIAN = 2, UNKNOWN = 127 };

struct Task {
    vector<UtmPosition> task_points;
};

static mutex config_mtx;

class Config {
public:
    const string TiEV_CONFIG_DIRECT = "/home/autolab/tiev2020-code/src/cfg/";
    // NOTICE: READ-ONLY
    string roadmap_file;
    // NOTICE: READ-ONLY
    ControlMode control_mode;
    // NOTICE: READ-ONLY
    HDMapMode debug_event_mode;
    // NOTICE: READ-ONLY
    HDMapSpeed debug_speed_mode;

    // NOTICE: READ-ONLY
    int targets_num_limit;
    // NOTICE: READ-ONLY
    int plan_time_limit_ms;
    // NOTICE: READ-ONLY
    int car_away_limit_meter;

    // NOTICE: READ-ONLY
    string rs_distance_table_path;
    // NOTICE: READ-ONLY
    string dubins_distance_table_path;
    // NOTICE: READ-ONLY
    double a_star_extention_step_meter;
    // NOTICE: READ-ONLY
    double a_star_analytic_expansion_param_k;
    // NOTICE: READ-ONLY
    double a_star_analytic_expansion_param_t;
    // NOTICE: READ-ONLY
    int a_star_analytic_expansion_max_N;
    // NOTICE: READ-ONLY
    double a_star_curvature_changed_punishment;
    // NOTICE: READ-ONLY
    vector<Task> tasks;

    static Config* getInstance() {
        config_mtx.lock();
        static Config inner_instance;
        config_mtx.unlock();
        return &inner_instance;
    }

private:
    Config() {
        init();
    }

    ~Config() {}

    void init();

    void outputConfigures() const;
};
}

#endif
