#include <unistd.h>
#include <iostream>
#include "map_manager.h"
#include "tiev_fsm.h"
#include "Routing.h"
namespace TiEV {
using namespace std;

void GlobalPlanning::enter(Control& control) {
  cout << "entry Global Planning..." << endl;
}

void GlobalPlanning::update(FullControl& control) {
  MapManager* map_manager = MapManager::getInstance();
  if(Config::getInstance()->taxi_mode){
    // 获取任务
    Routing* routing = Routing::getInstance();
    Task next = routing->waitForNextTask();
    map_manager->pushCurrentTask(next);
  }
  // cout << "Global Planning update..." << endl;
  usleep(20 * 1000);
  // for test
  map_manager->updateRefPath();
  Map& map = map_manager->getMap();
  if (!map.nav_info.detected || map_manager->getForwardRefPath().empty())
    return;
  ControlMode control_mode = Config::getInstance()->control_mode;
  if (control_mode == ControlMode::PlanningWithDebugMode ||
      control_mode == ControlMode::PlanningWithMapMode)
    control.changeTo<NormalDriving>();
  else
    control.changeTo<Tracking>();
  // TODO: map_manager->runRouting();
}
}  // namespace TiEV
