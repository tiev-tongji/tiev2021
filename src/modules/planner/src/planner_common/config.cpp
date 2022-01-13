#include "config.h"

#include <algorithm>
#include <fstream>
#include <iostream>

#include "rapidjson/document.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "tiev_utils.h"
#include "tievlog.h"

using namespace rapidjson;

namespace TiEV {

void Config::init() {
  ReadTasks(TiEV_CONFIG_DIRECT + "task_points.txt");

  std::string  para_file_path = TiEV_CONFIG_DIRECT + "plannerPara.json";
  std::fstream input(para_file_path, std::ios::in);
  if (!input.is_open()) {
    LOG(ERROR) << "Config file is not existed: " << para_file_path;
  }
  std::stringstream buffer;
  Document          doc;
  buffer << input.rdbuf();
  doc.Parse(buffer.str().c_str());
#define nameof(x) (#x)
  roadmap_file             = doc[nameof(roadmap_file)].GetString();
  auto& recommended_speeds = doc["recommended_speeds"];
  back_speed               = recommended_speeds["BACK"].Get<double>();
  stop_speed               = recommended_speeds["STOP"].Get<double>();
  very_low_speed           = recommended_speeds["VERYLOW"].Get<double>();
  low_speed                = recommended_speeds["LOW"].Get<double>();
  mid_speed                = recommended_speeds["MID"].Get<double>();
  high_speed               = recommended_speeds["HIGH"].Get<double>();
  very_high_speed          = recommended_speeds["VERYHIGH"].Get<double>();
  control_mode             = (ControlMode)(doc[nameof(control_mode)].GetInt());
  enable_routing_by_file   = (bool)doc[nameof(enable_routing_by_file)].GetInt();
  debug_speed_mode = (HDMapSpeed)(doc[nameof(debug_speed_mode)].GetInt());
  taxi_mode        = (bool)doc["taxi_mode"].GetInt();

  targets_num_limit      = doc[nameof(targets_num_limit)].Get<int>();
  plan_time_limit_ms     = doc[nameof(plan_time_limit_ms)].Get<int>();
  car_away_limit_meter   = doc[nameof(car_away_limit_meter)].Get<double>();
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
  host                 = routing_config["host"].GetString();
  port                 = routing_config["port"].GetString();
  dbname               = routing_config["dbname"].GetString();
  user                 = routing_config["user"].GetString();
  password             = routing_config["password"].GetString();
  topo_name            = routing_config["topo_name"].GetString();
  output               = routing_config["output"].GetString();

  auto tasks_arr = doc["task_sequence"].GetArray();
  for (int k = 0; k < tasks_arr.Size(); ++k) {
    auto task_name = tasks_arr[k].GetString();
    if (tasks_map.find(task_name) == tasks_map.end()) {
      LOG(FATAL) << "No task named \"" << task_name
                 << "\" in task points file! Please check task_sequence in "
                    "planner.json";
    }
    tasks.push_back(tasks_map[task_name]);
    // the fist task is to pick up the passenger
    tasks.back().get_on = (k + 1) % 2;
  }
  std::reverse(tasks.begin(), tasks.end());
  total_task_time        = doc["total_task_time"].GetDouble();
  auto parking_task_name = doc["parking_task_name"].GetString();
  if (tasks_map.find(parking_task_name) == tasks_map.end()) {
    LOG(FATAL) << "No task named \"" << parking_task_name
               << "\" in task points file! Please check parking_task_name in "
                  "planner.json";
  }
  parking_task = tasks_map[parking_task_name];
  start_time   = getTimeStamp();
  end_time     = start_time + total_task_time * 60 * 1e6;

#undef nameof
  input.close();
  outputConfigures();
}

void Config::outputConfigures() const {
  LOG(WARNING) << "--------------------";
  LOG(WARNING) << "Config";
  LOG(WARNING) << "--------------------";

#define print(a) LOG(INFO) << #a << " = " << (a)
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
  print(total_task_time);
  LOG(WARNING) << "---------All Task---------";
  for (auto& task : tasks) {
    LOG(INFO) << "------- task name=" << task.name << " get_on=" << task.get_on
              << " -------";
    for (auto& point : task.task_points) LOG(INFO) << point;
  }
  LOG(WARNING) << "-----------parking task name=" << parking_task.name
               << "-----------";
  for (const auto& point : parking_task.task_points) LOG(INFO) << point;
  LOG(WARNING) << "----------------------";
#undef print
}

std::string& Config::Trim(std::string& s) {
  if (s.empty()) {
    return s;
  }
  s.erase(0, s.find_first_not_of(" \r\n\t"));
  s.erase(s.find_last_not_of(" \r\n\t") + 1);
  return s;
}

void Config::SplitString(const std::string& s, std::vector<std::string>& v,
                         const std::string& c) {
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  while (std::string::npos != pos2) {
    v.push_back(s.substr(pos1, pos2 - pos1));

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length()) {
    v.push_back(s.substr(pos1));
  }
}
void Config::ReadTasks(const std::string& file_name) {
  std::ifstream fin(file_name.c_str());
  if (!fin.is_open()) {
    LOG(ERROR) << "Task file is not existed: " << file_name;
  }
  std::string line;
  while (std::getline(fin, line)) {
    line.erase(line.length() - line.find_first_not_of("#"));
    std::vector<std::string> items;
    SplitString(line, items, " ");
    std::string task_name = items[0];
    Trim(task_name);
    if (task_name == "Id") {
      continue;
    } else if (items.size() == 6) {
      double lon     = std::atof(items[1].c_str());
      double lat     = std::atof(items[2].c_str());
      double utm_x   = std::atof(items[3].c_str());
      double utm_y   = std::atof(items[4].c_str());
      double heading = std::atof(items[5].c_str());

      if (tasks_map.find(task_name) == tasks_map.end()) {
        tasks_map[task_name] = Task(task_name);
      }
      tasks_map[task_name].task_points.emplace_back(lon, lat, utm_x, utm_y,
                                                    heading);
    }
  }
  fin.close();
}

// Config Config::inner_instance;
}  // namespace TiEV
