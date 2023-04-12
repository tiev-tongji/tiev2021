#ifndef _MESSAGE_MANAGER_H
#define _MESSAGE_MANAGER_H

#include <map>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <vector>
#include <zcm/zcm-cpp.hpp>

#include "tiev_class.h"
#include "tievmsg.h"

class MessageManager {
public:
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

    void publishRemoteControl(const structREMOTECONTROL& remote_control);
    void publishSlamControl(const structSLAMCONTROL& slam_control);

private:
    class Handler {
    public:
        void handleNAVINFO(const zcm::ReceiveBuffer* rbuf,
                           const std::string& chan, const structNAVINFO* msg);
        void handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf,
                             const std::string&        chan,
                             const structFUSIONMAP*    msg);
        void handleRAINSIGNAL(const zcm::ReceiveBuffer*     rbuf,
                              const std::string&            chan,
                              const MsgRainDetectionSignal* msg);
        void handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf,
                              const std::string&        chan,
                              const structOBJECTLIST*   msg);
        void handleTRAFFICLIGHT(const zcm::ReceiveBuffer*    rbuf,
                                const std::string&           chan,
                                const MsgTrafficLightSignal* msg);
        void handleLANES(const zcm::ReceiveBuffer*    rbuf,
                         const std::string&           chan,
                         const structRoadMarkingList* msg);
        void handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf,
                                const std::string&        chan,
                                const structPARKINGSLOTS* msg);
        void handleSLAMLOC(const zcm::ReceiveBuffer* rbuf,
                           const std::string& chan, const structSLAMLOC* msg);
        void handleCANINFO(const zcm::ReceiveBuffer* rbuf,
                           const std::string& chan, const structCANINFO* msg);

        std::shared_mutex nav_mtx, map_mtx, objects_mtx, traffic_mtx,
            stop_line_mtx, lane_mtx, parking_slot_mtx, slam_loc_mtx, rain_mtx;

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
    zcm::ZCM zcm_ipc{ "ipc" };
    zcm::ZCM zcm_udp{ "ipc" };

public:
    MessageManager(const MessageManager&) = delete;
    MessageManager& operator=(const MessageManager&) = delete;

    static MessageManager& getInstance() {
        static MessageManager instance;
        return instance;
    }

private:
    MessageManager(){};
    ~MessageManager(){};
};

static std::time_t getTimeStamp() {
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::microseconds>
        tp = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::microseconds>(
        tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    // std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

static void normalizeAngle(double& rad_angle) {
    while(rad_angle > M_PI)
        rad_angle -= 2 * M_PI;
    while(rad_angle <= -M_PI)
        rad_angle += 2 * M_PI;
}

#endif
