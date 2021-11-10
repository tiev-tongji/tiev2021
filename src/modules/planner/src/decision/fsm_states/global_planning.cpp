#include <unistd.h>

#include <iostream>

#include "Routing.h"
#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
namespace TiEV {
using namespace std;

void GlobalPlanning::enter(Control& control) {
  cout << "entry Global Planning..." << endl;
}

void GlobalPlanning::update(FullControl& control) {
  MapManager* map_manager = MapManager::getInstance();
  if (Config::getInstance()->taxi_mode) {
    // 获取任务
    Routing* routing = Routing::getInstance();
    routing->updateInfoToServer();
    if (map_manager->getCurrentTasks().size() == 0) {
      Task next = routing->waitForNextTask();
      map_manager->pushCurrentTask(next);
    }
  }
  // for test
  map_manager->updateRefPath();
  const auto& map = map_manager->getMap();
  if (!map.nav_info.detected || map_manager->getForwardRefPath().empty()) {
    // LOG(WARNING) << "no reference path...";
    return;
  }
  ControlMode control_mode = Config::getInstance()->control_mode;
  if (control_mode == ControlMode::PlanningWithDebugMode ||
      control_mode == ControlMode::PlanningWithMapMode)
    control.changeTo<NormalDriving>();
    // control.changeTo<OvertakeDriving>();
  else
    control.changeTo<Tracking>();
}
}  // namespace TiEV
