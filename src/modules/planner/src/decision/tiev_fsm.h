#ifndef TiEV_FSM_H_
#define TiEV_FSM_H_
#include <vector>

#include "machine.hpp"
#include "map_manager.h"
#include "message_manager.h"
#include "path_planner.h"
#include "point2d.h"
#include "pose.h"

namespace TiEV {

//---------------------------------------------------
// data shared between FSM states and outside code
//请把状态中需要从外部更新的信息放在这里
struct Context {
  NavInfo        nav_info;
  SlamInfo       slam_info;
  TrafficLight   traffic_light;
  DynamicObjList dynamic_obj_list;
  WarningObjList warning_obj_list;
  ParkingLotList parking_lot_list;
  LaneList       lane_list;
  void           update();
};

// convenience typedef
using M = hfsm2::MachineT<hfsm2::Config::ContextT<Context>>;

//请准确构造途灵的层次状态机
#define S(s) struct s
// state machine structure
using FSM = M::PeerRoot<
    // OnRoad Fsm
    M::Composite<S(OnRoadFSM),
                 // substates
                 S(GlobalPlanning),   //
                 S(NormalDriving),    //
                 S(OvertakeDriving),  //
                 S(FreeDriving),      //
                 S(GlobalReplanning)  //
                 >,
    // Intersection Fsm
    M::Composite<S(IntersectionFSM),
                 // substates
                 S(IntersectionDriving)  //
                 >,                      //
    M::Composite<S(ParkingFSM),
                 // substates
                 S(ParkingPlanning)  //
                 >,                  //
    M::Composite<S(TemporaryParkingFSM),
                 // substates
                 S(TemporaryParkingPlanning),  //
                 S(TemporaryStop)              //
                 >,                            //
    S(Tracking)
    ,S(CloudTracking)
    >;

#undef S

//定义途灵中的状态机或状态，一定要和最上方状态机声明一致
//----------------OnRoad Fsm--------------------
struct TiEVState : FSM::State {
  time_t       entry_time;
  time_t       limited_time;
  const time_t duration_time() const { return getTimeStamp() - entry_time; }
};
struct OnRoadFSM : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

struct GlobalPlanning : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

struct NormalDriving : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

struct OvertakeDriving : TiEVState {
  void                    enter(Control& control);
  void                    update(FullControl& control);
  std::vector<HDMapPoint> overtakeLaneDecision(const Map& decision_map);
};

struct FreeDriving : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
  Pose previous_pose;
};

struct GlobalReplanning : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};
//----------------OnRoad Fsm--------------------
//----------------Intersection Fsm--------------------
struct IntersectionFSM : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

struct IntersectionDriving : TiEVState {
  void   enter(Control& control);
  void   update(FullControl& control);
  time_t last_block_time;
  time_t expired_time;
};

//----------------Intersection Fsm--------------------
//----------------Parking Fsm--------------------
struct ParkingFSM : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

struct ParkingPlanning : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

//----------------Parking Fsm--------------------
//----------------TemporaryParking Fsm--------------------
struct TemporaryParkingFSM : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

struct TemporaryParkingPlanning : TiEVState {
  void              enter(Control& control);
  void              update(FullControl& control);
  std::vector<Pose> planned_path_segment_idx;
  Pose              sub_target;
  std::vector<Pose> planned_path;
};

struct TemporaryStop : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

//----------------TemporaryParking Fsm--------------------
//----------------Tracking State--------------------
struct Tracking : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};


struct CloudTracking : TiEVState {
  void enter(Control& control);
  void update(FullControl& control);
};

//----------------Tracking State--------------------

}  // namespace TiEV

#endif