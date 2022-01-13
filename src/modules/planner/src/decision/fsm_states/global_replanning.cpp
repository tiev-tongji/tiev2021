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
  Task        current_pos;
  const auto  ref_car_point        = map_m.getForwardRefPath().front();
  current_pos.lon_lat_position.lon = ref_car_point.lon_lat_position.lon;
  current_pos.lon_lat_position.lat = ref_car_point.lon_lat_position.lat;
  std::vector<Task> task_list;
  task_list.push_back(current_pos);
  std::vector<Task> current_tasks = map_m.getCurrentTasks();
  if (!current_tasks.empty())
    task_list.push_back(current_tasks.back());
  else
    task_list.push_back(map_m.getParkingTask());
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
