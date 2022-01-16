#include <unistd.h>

#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {

void TemporaryStop::enter(Control& control) {
  LOG(INFO) << "entry Temporary Stop...";
  entry_time   = getTimeStamp();
  limited_time = 5e6;
}

void TemporaryStop::update(FullControl& control) {
  LOG(INFO) << "Temporary Stop update...";
  MapManager& map_manager = MapManager::getInstance();
  const auto  map         = map_manager.getMap();
  if (!map.nav_info.detected || duration_time() < limited_time) return;

  map_manager.popCurrentTask();
  bool      flag_parking  = false;
  auto      current_tasks = map_manager.getCurrentTasks();
  TaskPoint start_task_point;
  start_task_point.lon     = map.nav_info.lon;
  start_task_point.lat     = map.nav_info.lat;
  start_task_point.utm_x   = map.nav_info.car_pose.utm_position.utm_x;
  start_task_point.utm_y   = map.nav_info.car_pose.utm_position.utm_y;
  start_task_point.heading = map.nav_info.car_pose.utm_position.heading;
  std::vector<TaskPoint>  task_list;
  std::vector<HDMapPoint> tmp_global_path;
  task_list.push_back(start_task_point);
  Routing& routing = Routing::getInstance();
  if (current_tasks.empty()) {
    flag_parking = true;
  } else if (current_tasks.back().get_on && current_tasks.size() > 1) {
    task_list.push_back(current_tasks.back().task_points.back());
    task_list.push_back(
        current_tasks[int(current_tasks.size()) - 2].task_points.back());
    task_list.push_back(map_manager.getParkingTask().task_points.back());
    int    cost     = routing.findReferenceRoad(tmp_global_path, task_list,
                                         false);  // TODO: *3 off-on-off-parking
    time_t now_time = getTimeStamp();
    time_t end_time = Config::getInstance().end_time;
    if (now_time + cost * 1e6 > end_time) {
      flag_parking = true;
    } else {
      std::vector<TaskPoint> new_task_list;
      new_task_list.push_back(start_task_point);
      new_task_list.push_back(current_tasks.back().task_points.back());
      int cost =
          routing.findReferenceRoad(tmp_global_path, new_task_list,
                                    false);  // TODO: *3 off-on-off-parking
      map_manager.setGlobalPath(tmp_global_path);
    }
  } else {
    task_list.push_back(current_tasks.back().task_points.back());
    int cost = routing.findReferenceRoad(tmp_global_path, task_list, false);
    // // TODO: *3 off-on-off-parking
    map_manager.setGlobalPath(tmp_global_path);
  }
  if (flag_parking) {
    map_manager.clearTask();
    std::vector<TaskPoint> new_task_list;
    new_task_list.push_back(start_task_point);
    new_task_list.push_back(map_manager.getParkingTask().task_points.back());
    int cost = routing.findReferenceRoad(tmp_global_path, new_task_list, false);
    map_manager.setGlobalPath(tmp_global_path);
  }
  if (duration_time() < 20e6) {
    usleep(20e6 - duration_time());
    LOG(INFO) << "waiting...";
  }
  control.changeTo<NormalDriving>();
}
}  // namespace TiEV
