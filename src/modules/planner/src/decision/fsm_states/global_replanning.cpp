#include "map_manager.h"
#include "tiev_fsm.h"
#include <iostream>
namespace TiEV {
using namespace std;

void GlobalReplanning::enter(Control& control) {
    cout << "entry Global Replanning..." << endl;
}

void GlobalReplanning::update(FullControl& control) {
    cout << "Global Replanning update..." << endl;
    MapManager* map_m   = MapManager::getInstance();
    Routing*    routing = Routing::getInstance();
    Task        current_pos;
    Map&        map                  = map_m->getMap();
    current_pos.lon_lat_position.lon = map.nav_info.lon;
    current_pos.lon_lat_position.lat = map.nav_info.lat;
    vector<Task> task_list;
    task_list.push_back(current_pos);
    vector<Task> current_tasks = map_m->getCurrentTasks();
    if(!current_tasks.empty())
        task_list.push_back(current_tasks.back());
    else
        task_list.push_back(map_m->getParkingTask());
    int                cost = -1;
    vector<HDMapPoint> tmp_global_path;
    if(task_list.size() > 1) cost = routing->findReferenceRoad(tmp_global_path, task_list, true);
    if(cost == -1) return;
    cout << "Cost of global path: " << cost << endl;
    map_m->setGlobalPath(tmp_global_path);
    control.changeTo<UTurn>();
}
}  // namespace TiEV
