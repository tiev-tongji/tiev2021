#ifndef _TIEV_PLANNER_CONFIG_H_
#define _TIEV_PLANNER_CONFIG_H_

#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include "pose.h"

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
  UtmPosition utm_position;
  LonLatPosition lon_lat_position;
  vector<UtmPosition> task_points;
  bool on_or_off;  // 0 is on, 1 is off
  Task(double utm_x_ = 0, double utm_y_ = 0, double lon_ = 0, double lat_ = 0,
       double heading_ = 0, bool on_or_off_ = 0)
      : utm_position(utm_x_, utm_y_, heading_),
        lon_lat_position(lon_, lat_, heading_),
        on_or_off(on_or_off_){};
};

static mutex config_mtx;

class Config {
 public:
  const string TiEV_CONFIG_DIRECT = "../../../cfg/";
  // NOTICE: READ-ONLY
  string roadmap_file;
  // NOTICE: READ-ONLY km/h
  double back_speed;
  double stop_speed;
  double very_low_speed;
  double low_speed;
  double mid_speed;
  double high_speed;
  double very_high_speed;

  time_t start_time;
  time_t end_time;
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
  Task parking_task;

  // Configuration of routing
  string host;
  string port;
  string dbname;
  string user;
  string password;
  string topo_name;
  string output;

  static Config* getInstance() {
    config_mtx.lock();
    static Config inner_instance;
    config_mtx.unlock();
    return &inner_instance;
  }

 private:
  Config() { init(); }

  ~Config() {}

  void init();

  void outputConfigures() const;
};
}  // namespace TiEV

#endif
