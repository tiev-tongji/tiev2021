#ifndef _MESSAGE_MANAGER_H
#define _MESSAGE_MANAGER_H

#include "config.h"
#include "speed_optimizer.h"
#include "tiev_class.h"
#include "tiev_utils.h"
#include "tievmsg.h"
#include "visualization_msg.h"
//#include <shared_mutex>
#include <thread>
#include <thread>
#include <tuple>
#include <vector>
#include <zcm/zcm-cpp.hpp>

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
    const static std::time_t TRAFFIC_LIGHT_TIMEOUT_US = 1e6;
    const static std::time_t PARKING_SLOT_TIMEOUT_US  = 1e6;
    const static std::time_t OBJECT_LIST_TIMEOUT_US   = 1e6;
    const static std::time_t SLAM_LOC_TIMEOUT_US      = 1e6;
    const static std::time_t CAN_INFO_TIMEOUT_US      = 1e5;
    const static int         OBJECTS_SOURCE_NUM       = 3;

    void msgReceiveIpc();
    void msgReceiveUdp();

    bool getNavInfo(NavInfo& nav_info);
    bool getMap(LidarMap& lidar_map);
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
    visVISUALIZATION visualization;
    void             publishVisualization();
    void setTargets(const vector<Pose>& targets);
    void setStartPoint(const Pose& start_point);
    void setSafeMap(double safe_map[MAX_ROW][MAX_COL]);
    void setPath(const vector<Pose>& path);
    void setUsedMap(bool used_map[MAX_ROW][MAX_COL]);
    void clearPaths();
    void setSpeedPath(const SpeedPath& speed_path);

private:
    MessageManager(){};
    ~MessageManager(){};

    class Handler {
    public:
        void handleNAVINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO* msg);
        void handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structFUSIONMAP* msg);
        void handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structOBJECTLIST* msg);
        void handleTRAFFICLIGHT(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structTRAFFICLIGHT* msg);
        void handleLANES(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structLANES* msg);
        void handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structPARKINGSLOTS* msg);
        void handleSLAMLOC(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structSLAMLOC* msg);
        void handleCANINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO* msg);

        TiEV::shared_mutex nav_mtx, map_mtx, objects_mtx, traffic_mtx, stop_line_mtx, lane_mtx, parking_slot_mtx, slam_loc_mtx;

        time_t update_time_nav_info;
        time_t update_time_fusion_map;
        time_t update_time_lane;
        time_t update_time_traffic_light;
        time_t update_time_objects[OBJECTS_SOURCE_NUM];
        time_t update_time_stop_line;
        time_t update_time_parking_slots;
        time_t update_time_slam_loc;
        time_t update_time_can_info;

        structNAVINFO      tmp_nav;
        structFUSIONMAP    tmp_map;
        structOBJECTLIST   tmp_objects[OBJECTS_SOURCE_NUM];
        structTRAFFICLIGHT tmp_traffic;
        structSLAMLOC      tmp_slam_loc;
        structCANINFO      tmp_can_info;
        structLANES        tmp_lanes;
        structPARKINGSLOTS tmp_slot;
    };

    Handler  inner_handler;
    zcm::ZCM zcm_ipc{ "ipc" };
    zcm::ZCM zcm_udp{ "" };
};
}

#endif
