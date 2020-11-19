#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void TemporaryStop::enter(Control& control) {
    cout << "entry Temporary Stop..." << endl;
    entry_time = getTimeStamp();
}

void TemporaryStop::update(FullControl& control) {
    cout << "Temporary Stop update..." << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    if(!map.nav_info.detected || getTimeStamp() - entry_time < 5e6) return;

    map_manager->popCurrentTask();
    bool flag_parking  = false;
    auto current_tasks = map_manager->getCurrentTasks();
    Task current_pos;
    current_pos.lon_lat_position.lon = map.nav_info.lon;
    current_pos.lon_lat_position.lat = map.nav_info.lat;
    vector<Task>       task_list;
    vector<HDMapPoint> tmp_global_path;
    task_list.push_back(current_pos);
    Routing* routing = Routing::getInstance();

    if(current_tasks.empty())
        flag_parking = true;
    else {
        task_list.push_back(current_tasks.back());
        int cost = routing->findReferenceRoad(tmp_global_path, task_list, false);  // TODO: *3 off-on-off-parking
        if(current_tasks.back().on_or_off) {                                       // On
            time_t now_time = getTimeStamp();
            time_t end_time = Config::getInstance()->end_time;
            if(now_time + cost * 1e6 > end_time) {
                flag_parking = true;
            }
        }
    }
    if(flag_parking) {
        Task parking_pos;
        // TODO: set parking pos
        task_list[1] = parking_pos;
        tmp_global_path.clear();
        int cost = routing->findReferenceRoad(tmp_global_path, task_list, false);
    }
    map_manager->setGlobalPath(tmp_global_path);
    control.changeTo<SemiLaneFreeDriving>();
}
}  // namespace TiEV
