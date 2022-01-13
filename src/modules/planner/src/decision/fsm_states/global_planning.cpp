#include <unistd.h>

#include <iostream>

#include "Routing.h"
#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {

void GlobalPlanning::enter(Control& control) {
  LOG(INFO) << "entry Global Planning...";
}

void GlobalPlanning::update(FullControl& control) {
  MapManager& map_manager = MapManager::getInstance();
  map_manager.updateRefPath();
  const auto& map = map_manager.getMap();
  if (!map.nav_info.detected) {
    LOG(WARNING) << "No Navinfo!";
    usleep(1e6);
    return;
  }
  if (!Config::getInstance().enable_routing_by_file) {
    Routing&  routing = Routing::getInstance();
    TaskPoint start_task_point;
    start_task_point.lon     = map.nav_info.lon;
    start_task_point.lat     = map.nav_info.lat;
    start_task_point.utm_x   = map.nav_info.car_pose.utm_position.utm_x;
    start_task_point.utm_y   = map.nav_info.car_pose.utm_position.utm_y;
    start_task_point.heading = map.nav_info.car_pose.utm_position.heading;
    std::vector<TaskPoint> task_list;
    task_list.push_back(start_task_point);
    std::vector<Task> current_tasks = map_manager.getCurrentTasks();
    if (!current_tasks.empty())
      task_list.push_back(current_tasks.back().task_points.back());
    else
      task_list.push_back(map_manager.getParkingTask().task_points.back());
    int                     cost = -1;
    std::vector<HDMapPoint> tmp_global_path;
    if (task_list.size() > 1) {
      cost = routing.findReferenceRoad(tmp_global_path, task_list, false);
    } else {
      LOG(WARNING) << "Please check the task points in file...";
      usleep(1e6);
      return;
    }
    if (cost < 0) {
      LOG(WARNING) << "Retry Requesting Remote Service...";
      return;
    }
    map_manager.setGlobalPath(tmp_global_path);
  }
  ControlMode control_mode = Config::getInstance().control_mode;
  if (control_mode == ControlMode::PlanningWithDebugMode ||
      control_mode == ControlMode::PlanningWithMapMode)
    control.changeTo<NormalDriving>();
  else
    control.changeTo<Tracking>();
}
}  // namespace TiEV
