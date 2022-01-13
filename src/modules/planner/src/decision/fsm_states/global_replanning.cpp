#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {

void GlobalReplanning::enter(Control& control) {
  LOG(INFO) << "entry Global Re-planning...";
}

void GlobalReplanning::update(FullControl& control) {
  LOG(INFO) << "Global Re-planning update...";
  MapManager& map_m   = MapManager::getInstance();
  Routing&    routing = Routing::getInstance();
  TaskPoint   start_task_point;
  const auto  ref_car_point = map_m.getForwardRefPath().front();
  start_task_point.lon      = ref_car_point.lon_lat_position.lon;
  start_task_point.lat      = ref_car_point.lon_lat_position.lat;
  start_task_point.utm_x    = ref_car_point.utm_position.utm_x;
  start_task_point.utm_y    = ref_car_point.utm_position.utm_y;
  start_task_point.heading  = ref_car_point.utm_position.heading;
  std::vector<TaskPoint> task_list;
  task_list.push_back(start_task_point);
  const auto current_tasks = map_m.getCurrentTasks();
  if (!current_tasks.empty())
    task_list.push_back(current_tasks.back().task_points.back());
  else
    task_list.push_back(map_m.getParkingTask().task_points.back());
  int                     cost = -1;
  std::vector<HDMapPoint> tmp_global_path;
  if (task_list.size() > 1)
    cost = routing.findReferenceRoad(tmp_global_path, task_list, true);
  if (cost < 0) {
    LOG(WARNING) << "Retry Re-planning...";
    return;
  }
  map_m.setGlobalPath(tmp_global_path);
  control.changeTo<NormalDriving>();
}
}  // namespace TiEV
