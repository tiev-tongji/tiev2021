#include "tiev_fsm.h"
namespace TiEV {
using namespace std;
void Context::update() {
    MessageManager* msgm = MessageManager::getInstance();
    msgm->getNavInfo(nav_info);
    msgm->getSlamInfo(slam_info);
    msgm->getTrafficLight(traffic_light);
    msgm->getDynamicObjList(dynamic_obj_list);
    msgm->getWarningObjList(warning_obj_list);
    msgm->getParkingLotList(parking_lot_list);
    msgm->getLaneList(lane_list);
    cout << "context information update!" << endl;
}

//----------------OnRoad Fsm--------------------
void OnRoadFSM::enter(Control& control) {
    std::cout << "Entry OnRoad Fsm..." << std::endl;
}

void OnRoadFSM::update(FullControl& control) {
    cout << "OnRoad Fsm update!" << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    if(!map.forward_ref_path.empty()) {
        auto mode = map.forward_ref_path.front().mode;
        if(mode == HDMapMode::INTERSECTION_SOLID || mode == HDMapMode::INTERSECTION)
            control.changeTo<SafeDriving>();
        else if(mode == HDMapMode::PARKING)
            control.changeTo<SeekParkingSpot>();
    }
    // TODO: temporary_parking
}
//----------------OnRoad Fsm--------------------

//----------------Intersection Fsm--------------------
void IntersectionFSM::enter(Control& control) {
    std::cout << "Entry Intersection FSM..." << std::endl;
}

void IntersectionFSM::update(FullControl& control) {
    cout << "Intersection FSM update!" << endl;
    MapManager* map_manager = MapManager::getInstance();
    Map&        map         = map_manager->getMap();
    if(!map.forward_ref_path.empty()) {
        auto mode = map.forward_ref_path.front().mode;
        if(mode == HDMapMode::NORMAL) control.changeTo<NormalDriving>();
    }
}
//----------------Intersection Fsm--------------------

//----------------Parking Fsm--------------------
void ParkingFSM::enter(Control& control) {
    std::cout << "Entry Parking FSM..." << std::endl;
}

void ParkingFSM::update(FullControl& control) {
    cout << "Parking FSM update!" << endl;
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
