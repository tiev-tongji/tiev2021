#ifndef _TIEV_PLANNER_CONFIG_H_
#define _TIEV_PLANNER_CONFIG_H_

#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "pose.h"

namespace TiEV {

enum ControlMode {
  PlanningWithMapMode,    //规划模式，使用地图RoadMode和SpeedMode
  TrakingWithMapMode,     //循迹模式，使用地图RoadMode和SpeedMode
  PlanningWithDebugMode,  //规划模式，使用debugRoadMode和debugSpeedMode
  TrakingWithDebugMode    //循迹模式，使用debugRoadMode和debugSpeedMode
};

enum ObjectType { CAR = 0, BYCICLE = 1, PEDESTRIAN = 2, UNKNOWN = 127 };

struct TaskPoint {
  double lon;
  double lat;
  double utm_x;
  double utm_y;
  double heading;

  TaskPoint(const double lon_ = 0, const double lat_ = 0,
            const double utm_x_ = 0, const double utm_y_ = 0,
            const double heading_ = 0)
      : lon(lon_), lat(lat_), utm_x(utm_x_), utm_y(utm_y_), heading(heading_) {}

  friend std::ostream &operator<<(std::ostream &out, const TaskPoint &point) {
    out << "TaskPoint:{" << point.lon << ", " << point.lat << ", "
        << point.utm_x << ", " << point.utm_y << ", " << point.heading << "}";
    return out;
  }
};

struct Task {
  std::string            name;
  bool                   get_on;  // 1 is on, 0 is off
  std::vector<TaskPoint> task_points;

  Task(const std::string name_ = "no_name", bool get_on_ = false)
      : name(name_), get_on(get_on_) {}
};

class Config {
 public:
  const std::string TiEV_CONFIG_DIRECT = "../../../cfg/";
  // NOTICE: READ-ONLY
  std::string roadmap_file;
  // NOTICE: READ-ONLY km/h
  double back_speed;
  double stop_speed;
  double very_low_speed;
  double low_speed;
  double mid_speed;
  double high_speed;
  double very_high_speed;

  double total_task_time;
  time_t start_time;
  time_t end_time;
  // NOTICE: READ-ONLY
  ControlMode control_mode;
  // NOTICE: READ-ONLY
  bool enable_routing_by_file;
  // NOTICE: READ-ONLY
  HDMapSpeed debug_speed_mode;
  // NOTICE: READ-ONLY
  bool taxi_mode;
  // NOTICE: READ-ONLY
  int targets_num_limit;
  // NOTICE: READ-ONLY
  int plan_time_limit_ms;
  // NOTICE: READ-ONLY
  int car_away_limit_meter;

  // NOTICE: READ-ONLY
  std::string rs_distance_table_path;
  // NOTICE: READ-ONLY
  std::string dubins_distance_table_path;
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
  std::unordered_map<std::string, Task> tasks_map;
  std::vector<Task>                     tasks;
  Task                                  parking_task;

  // Configuration of routing
  std::string host;
  std::string port;
  std::string dbname;
  std::string user;
  std::string password;
  std::string topo_name;
  std::string output;

 public:
  Config(const Config &) = delete;
  Config &operator=(const Config &) = delete;

  static const Config &getInstance() {
    static Config instance;
    return instance;
  }

 private:
  Config() { init(); }

  ~Config() {}

  void init();

  void outputConfigures() const;

  std::string &Trim(std::string &);

  void SplitString(const std::string &, std::vector<std::string> &,
                   const std::string &);

  void ReadTasks(const std::string &);
};
}  // namespace TiEV

#endif
