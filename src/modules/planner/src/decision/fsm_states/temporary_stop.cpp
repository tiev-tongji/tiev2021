#include <unistd.h>

#include <iostream>

#include "map_manager.h"
#include "tiev_fsm.h"
namespace TiEV {
using namespace std;

void TemporaryStop::enter(Control& control) {
  cout << "entry Temporary Stop..." << endl;
  entry_time   = getTimeStamp();
  limited_time = 5e6;
}

void TemporaryStop::update(FullControl& control) {
  cout << "Temporary Stop update..." << endl;
  MapManager& map_manager = MapManager::getInstance();
  const auto  map         = map_manager.getMap();
  if (!map.nav_info.detected || duration_time() < limited_time) return;

  map_manager.popCurrentTask();
  bool flag_parking  = false;
  auto current_tasks = map_manager.getCurrentTasks();
  Task current_pos;
  current_pos.lon_lat_position.lon = map.nav_info.lon;
  current_pos.lon_lat_position.lat = map.nav_info.lat;
  vector<Task>       task_list;
  vector<HDMapPoint> tmp_global_path;
  task_list.push_back(current_pos);
  Routing& routing = Routing::getInstance();
  if (current_tasks.empty()) {
    flag_parking = true;
  } else if (current_tasks.back().get_on && current_tasks.size() > 1) {
    task_list.push_back(current_tasks.back());
    task_list.push_back(current_tasks[int(current_tasks.size()) - 2]);
    task_list.push_back(map_manager.getParkingTask());
    int    cost     = routing.findReferenceRoad(tmp_global_path, task_list,
                                         false);  // TODO: *3 off-on-off-parking
    time_t now_time = getTimeStamp();
    time_t end_time = Config::getInstance().end_time;
    if (now_time + cost * 1e6 > end_time) {
      flag_parking = true;
    } else {
      vector<Task> new_task_list;
      new_task_list.push_back(current_pos);
      new_task_list.push_back(current_tasks.back());
      int cost =
          routing.findReferenceRoad(tmp_global_path, new_task_list,
                                    false);  // TODO: *3 off-on-off-parking
      map_manager.setGlobalPath(tmp_global_path);
    }
  } else {
    task_list.push_back(current_tasks.back());
    int cost = routing.findReferenceRoad(tmp_global_path, task_list, false);
    // // TODO: *3 off-on-off-parking
    map_manager.setGlobalPath(tmp_global_path);
  }
  if (flag_parking) {
    map_manager.clearTask();
    vector<Task> new_task_list;
    new_task_list.push_back(current_pos);
    new_task_list.push_back(map_manager.getParkingTask());
    int cost = routing.findReferenceRoad(tmp_global_path, new_task_list, false);
    map_manager.setGlobalPath(tmp_global_path);
  }
  if (duration_time() < 20e6) usleep(20e6 - duration_time());
  control.changeTo<NormalDriving>();
}
}  // namespace TiEV
