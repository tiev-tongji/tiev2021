#include "tiev_fsm.h"

#include "decision_context.h"
namespace TiEV {
using namespace std;
void Context::update() {
  MessageManager& msgm = MessageManager::getInstance();
  /*
  msgm.getNavInfo(nav_info);
  msgm.getSlamInfo(slam_info);
  msgm.getTrafficLight(traffic_light);
  msgm.getDynamicObjList(dynamic_obj_list);
  msgm.getWarningObjList(warning_obj_list);
  msgm.getParkingLotList(parking_lot_list);
  msgm.getLaneList(lane_list);
  */
  // cout << "context information update!" << endl;
}

//----------------OnRoad Fsm--------------------
void OnRoadFSM::enter(Control& control) {
  std::cout << "Entry OnRoad Fsm..." << std::endl;
}

void OnRoadFSM::update(FullControl& control) {
  // cout << "OnRoad Fsm update!" << endl;
  MapManager& map_manager   = MapManager::getInstance();
  Map&        map           = map_manager.getMap();
  Pose        car_pose      = map.nav_info.car_pose;
  auto        current_tasks = map_manager.getCurrentTasks();
  auto        parking_task  = map_manager.getParkingTask();
  double      sqr_dis       = 1e9;
  if (!control.isActive<TemporaryParkingPlanning>() &&
      (!current_tasks.empty() || !parking_task.task_points.empty())) {
    UtmPosition task_utm;
    if (!current_tasks.empty())
      task_utm = current_tasks.back().utm_position;
    else
      task_utm = parking_task.utm_position;
    Pose task_pose;
    task_pose.utm_position = task_utm;
    task_pose.updateLocalCoordinate(map.nav_info.car_pose);
    // cout << "The task: " << task_pose << " car pose:" <<
    // map.nav_info.car_pose << endl;
  }
  auto mode = map_manager.getCurrentMapMode();
  if (mode == HDMapMode::INTERSECTION_SOLID ||
      mode == HDMapMode::INTERSECTION) {
    control.changeTo<IntersectionDriving>();
  }
  // else if (mode == HDMapMode::PARKING)
  //   control.changeTo<SeekParkingSpot>();
}
//----------------OnRoad Fsm--------------------

//----------------Intersection Fsm--------------------
void IntersectionFSM::enter(Control& control) {
  std::cout << "Entry Intersection FSM..." << std::endl;
}

void IntersectionFSM::update(FullControl& control) {
  cout << "Intersection FSM update!" << endl;
  MapManager& map_manager = MapManager::getInstance();
  auto        mode        = map_manager.getCurrentMapMode();
  if (mode != HDMapMode::INTERSECTION_SOLID &&
      mode != HDMapMode::INTERSECTION) {
    // erase decision context in intersectionFSM before going to another FSM
    // state
    auto&                   decision_context = DecisionContext::getInstance();
    std::vector<DynamicObj> empty_objlist;
    decision_context.setPedestrianDecision(empty_objlist);
    control.changeTo<NormalDriving>();
  }
}
//----------------Intersection Fsm--------------------

//----------------Parking Fsm--------------------
void ParkingFSM::enter(Control& control) {
  std::cout << "Entry Parking FSM..." << std::endl;
}

void ParkingFSM::update(FullControl& control) {
  cout << "Parking FSM update!" << endl;
  MapManager& map_manager = MapManager::getInstance();
  auto        mode        = map_manager.getCurrentMapMode();
  if (mode != HDMapMode::PARKING) control.changeTo<NormalDriving>();
}
//----------------Parking Fsm--------------------

//----------------TemporaryParking Fsm--------------------
void TemporaryParkingFSM::enter(Control& control) {
  std::cout << "Entry TemporaryParking FSM..." << std::endl;
}

void TemporaryParkingFSM::update(FullControl& control) {
  cout << "TemporaryParking FSM update!" << endl;
}
//----------------TemporaryParking Fsm--------------------
}  // namespace TiEV
