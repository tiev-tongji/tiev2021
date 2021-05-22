#include "config.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include "rapidjson/document.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "tiev_utils.h"

using namespace rapidjson;
using namespace std;

namespace TiEV {

void Config::init() {
  string para_file_path = TiEV_CONFIG_DIRECT + "plannerPara.json";
  fstream input(para_file_path, ios::in);
  if (!input.is_open()) {
    throw "Open config failed: file not exist.";
  }
  stringstream buffer;
  Document doc;
  buffer << input.rdbuf();
  doc.Parse(buffer.str().c_str());
#define nameof(x) (#x)
  roadmap_file = doc[nameof(roadmap_file)].GetString();
  auto& recommended_speeds = doc["recommended_speeds"];
  back_speed = recommended_speeds["BACK"].Get<double>();
  stop_speed = recommended_speeds["STOP"].Get<double>();
  very_low_speed = recommended_speeds["VERYLOW"].Get<double>();
  low_speed = recommended_speeds["LOW"].Get<double>();
  mid_speed = recommended_speeds["MID"].Get<double>();
  high_speed = recommended_speeds["HIGH"].Get<double>();
  very_high_speed = recommended_speeds["VERYHIGH"].Get<double>();
  control_mode = (ControlMode)(doc[nameof(control_mode)].GetInt());
  enable_routing_by_file = (bool)doc[nameof(enable_routing_by_file)].GetInt();
  debug_speed_mode = (HDMapSpeed)(doc[nameof(debug_speed_mode)].GetInt());
  taxi_mode = (bool)doc["taxi_mode"].GetInt();
  targets_num_limit = doc[nameof(targets_num_limit)].Get<int>();
  plan_time_limit_ms = doc[nameof(plan_time_limit_ms)].Get<int>();
  car_away_limit_meter = doc[nameof(car_away_limit_meter)].Get<double>();
  rs_distance_table_path = doc[nameof(rs_distance_table_path)].GetString();
  dubins_distance_table_path =
      doc[nameof(dubins_distance_table_path)].GetString();
  a_star_extention_step_meter =
      doc[nameof(a_star_extention_step_meter)].Get<double>();
  a_star_analytic_expansion_param_k =
      doc[nameof(a_star_analytic_expansion_param_k)].Get<double>();
  a_star_analytic_expansion_param_t =
      doc[nameof(a_star_analytic_expansion_param_t)].Get<double>();
  a_star_analytic_expansion_max_N =
      doc[nameof(a_star_analytic_expansion_max_N)].Get<int>();
  auto& routing_config = doc["routing"];
  host = routing_config["host"].GetString();
  port = routing_config["port"].GetString();
  dbname = routing_config["dbname"].GetString();
  user = routing_config["user"].GetString();
  password = routing_config["password"].GetString();
  topo_name = routing_config["topo_name"].GetString();
  output = routing_config["output"].GetString();
  auto& parking_task_pos = doc["parking_task"];
  parking_task.lon_lat_position.lon = parking_task_pos["lon"].GetDouble();
  parking_task.lon_lat_position.lat = parking_task_pos["lat"].GetDouble();
  parking_task.utm_position.utm_x = parking_task_pos["utm_x"].GetDouble();
  parking_task.utm_position.utm_y = parking_task_pos["utm_y"].GetDouble();
  auto parking_task_points = parking_task_pos["task_points"].GetArray();
  parking_task.task_points.resize(parking_task_points.Size());
  for (int k = 0; k < parking_task_points.Size(); ++k) {
    double utm_x = parking_task_points[k]["utm_x"].GetDouble();
    double utm_y = parking_task_points[k]["utm_y"].GetDouble();
    double heading = parking_task_points[k]["heading"].GetDouble();
    parking_task.task_points[k] = UtmPosition(utm_x, utm_y, heading);
  }

  // start_time = doc["start_time"].GetInt64();
  start_time = getTimeStamp();
  end_time = start_time + 1e6 * 60 * 60 * 10;

  tasks.clear();
  auto task_arr = doc["tasks"].GetArray();
  tasks.resize(task_arr.Size());
  for (int i = 0; i < task_arr.Size(); ++i) {
    auto& task = task_arr[i]["task"];
    tasks[i].lon_lat_position.lon = task["lon"].GetDouble();
    tasks[i].lon_lat_position.lat = task["lat"].GetDouble();
    tasks[i].utm_position.utm_x = task["utm_x"].GetDouble();
    tasks[i].utm_position.utm_y = task["utm_y"].GetDouble();

    auto task_points_arr = task_arr[i]["task_points"].GetArray();
    tasks[i].task_points.resize(task_points_arr.Size());
    for (int j = 0; j < task_points_arr.Size(); ++j) {
      double utm_x = task_points_arr[j]["utm_x"].GetDouble();
      double utm_y = task_points_arr[j]["utm_y"].GetDouble();
      double heading = task_points_arr[j]["heading"].GetDouble();
      tasks[i].task_points[j] = UtmPosition(utm_x, utm_y, heading);
    }
    tasks[i].on_or_off = task["on"].GetInt();
  }
  reverse(tasks.begin(), tasks.end());
#undef nameof
  input.close();
  outputConfigures();
}

void Config::outputConfigures() const {
  cout << "--------------------" << endl
       << "Config" << endl
       << "--------------------" << endl;

#define print(a) cout << #a << " = " << (a) << endl
  print(roadmap_file);
  print(back_speed);
  print(stop_speed);
  print(very_low_speed);
  print(low_speed);
  print(mid_speed);
  print(high_speed);
  print(very_high_speed);
  print(rs_distance_table_path);
  print(dubins_distance_table_path);
  print(control_mode);
  print(enable_routing_by_file);
  print(debug_speed_mode);
  print(targets_num_limit);
  print(plan_time_limit_ms);
  print(car_away_limit_meter);
  print(a_star_extention_step_meter);
  print(a_star_analytic_expansion_param_k);
  print(a_star_analytic_expansion_param_t);
  print(a_star_analytic_expansion_max_N);
  print(tasks.size());
  cout << "start_time" << start_time << endl;
  cout << "parking task:" << parking_task.utm_position << endl;
  cout << "parking task points:" << endl;
  for (const auto& point : parking_task.task_points)
    cout << "utm(" << point.utm_x << "," << point.utm_y << "," << point.heading
         << ") " << endl;
  for (auto& task : tasks) {
    cout << "- ";
    cout << "task utm position: " << task.utm_position << endl;
    cout << "task lonlat position: " << task.lon_lat_position.lon << " "
         << task.lon_lat_position.lat << endl;
    cout << "task on or off: " << task.on_or_off << endl;
    cout << "- ";
    for (auto& point : task.task_points)
      cout << "utm(" << point.utm_x << "," << point.utm_y << ","
           << point.heading << ") ";
    cout << endl;
  }
#undef print
}

// Config Config::inner_instance;
}  // namespace TiEV
