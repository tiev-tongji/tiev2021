#ifndef _MESSAGE_MANAGER_H
#define _MESSAGE_MANAGER_H

#include <map>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <vector>
#include <zcm/zcm-cpp.hpp>

#include "config.h"
#include "speed_optimizer.h"
#include "tiev_class.h"
#include "tiev_utils.h"
#include "tievmsg.h"
#include "visualization_msg.h"

namespace TiEV {

static mutex msg_manager_mtx;
class MessageManager {
 public:
  static MessageManager* getInstance() {
    msg_manager_mtx.lock();
    static MessageManager instance;
    msg_manager_mtx.unlock();
    return &instance;
  }

  const static std::time_t NAV_INFO_TIMEOUT_US      = 1e6;
  const static std::time_t LIDAR_MAP_TIMEOUT_US     = 1e6;
  const static std::time_t LANE_TIMEOUT_US          = 2e5;
  const static std::time_t TRAFFIC_LIGHT_TIMEOUT_US = 2e6;
  const static std::time_t PARKING_SLOT_TIMEOUT_US  = 1e6;
  const static std::time_t OBJECT_LIST_TIMEOUT_US   = 1e6;
  const static std::time_t SLAM_LOC_TIMEOUT_US      = 1e6;
  const static std::time_t CAN_INFO_TIMEOUT_US      = 1e5;
  const static std::time_t RAIN_SIGNAL_TIMEOUT_US   = 1e5;
  const static int         OBJECTS_SOURCE_NUM       = 3;

  void msgReceiveIpc();
  void msgReceiveUdp();

  bool getNavInfo(NavInfo& nav_info);
  bool getMap(LidarMap& lidar_map);
  bool getRainSignal(RainSignal& rain_signal);
  bool getDynamicObjList(DynamicObjList& dynamic_obj_list);
  bool getWarningObjList(WarningObjList& warning_obj_list);
  bool getTrafficLight(TrafficLight& traffic_light);
  bool getLaneList(LaneList& lane_list);
  bool getParkingLotList(ParkingLotList& parking_lot_list);
  bool getPedestrian(Pedestrian& pedestrian);
  bool getSlamInfo(SlamInfo& slam_info);

  void publishPath(const structAIMPATH& path);
  void publishRemoteControl(const structREMOTECONTROL& remote_control);
  void publishSlamControl(const structSLAMCONTROL& slam_control);

  // 用于决策与规划等模块发送可视化信息至visualization
  visVISUALIZATION    visualization;
  map<string, string> text_info;

  void publishVisualization();
  void setTarget(const Pose& target);
  void setStartPoint(const Pose& start_point);
  void setSafeMap(double safe_map[MAX_ROW][MAX_COL]);
  void setPath(const vector<Pose>& path);
  void setUsedMap(bool used_map[MAX_ROW][MAX_COL]);
  void setSpeedPath(const SpeedPath& speed_path);
  void clearTextInfo();
  void addTextInfo(const string& name, const string& value);
  void setTextInfo();
  void setPriorityLane(const std::vector<HDMapPoint>& ref_path,
                       const std::vector<Point2d>&    origin_points);

 private:
  MessageManager(){};
  ~MessageManager(){};

  class Handler {
   public:
    void handleNAVINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const structNAVINFO* msg);
    void handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf,
                         const std::string& chan, const structFUSIONMAP* msg);
    void handleRAINSIGNAL(const zcm::ReceiveBuffer*     rbuf,
                          const std::string&            chan,
                          const MsgRainDetectionSignal* msg);
    void handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf,
                          const std::string& chan, const structOBJECTLIST* msg);
    void handleTRAFFICLIGHT(const zcm::ReceiveBuffer*    rbuf,
                            const std::string&           chan,
                            const MsgTrafficLightSignal* msg);
    void handleLANES(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const structRoadMarkingList* msg);
    void handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf,
                            const std::string&        chan,
                            const structPARKINGSLOTS* msg);
    void handleSLAMLOC(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const structSLAMLOC* msg);
    void handleCANINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const structCANINFO* msg);

    std::shared_mutex nav_mtx, map_mtx, objects_mtx, traffic_mtx, stop_line_mtx,
        lane_mtx, parking_slot_mtx, slam_loc_mtx, rain_mtx;

    time_t update_time_nav_info;
    time_t update_time_fusion_map;
    time_t update_time_lane;
    time_t update_time_traffic_light;
    time_t update_time_objects[OBJECTS_SOURCE_NUM];
    time_t update_time_stop_line;
    time_t update_time_parking_slots;
    time_t update_time_slam_loc;
    time_t update_time_can_info;
    time_t update_time_rain_signal;

    structNAVINFO          tmp_nav;
    structFUSIONMAP        tmp_map;
    MsgRainDetectionSignal tmp_rain_signal;
    structOBJECTLIST       tmp_objects[OBJECTS_SOURCE_NUM];
    MsgTrafficLightSignal  tmp_traffic;
    structSLAMLOC          tmp_slam_loc;
    structCANINFO          tmp_can_info;
    structRoadMarkingList  tmp_lanes;
    structPARKINGSLOTS     tmp_slot;
  };

  Handler  inner_handler;
  zcm::ZCM zcm_ipc{"ipc"};
  zcm::ZCM zcm_udp{""};
};
}  // namespace TiEV

#endif
