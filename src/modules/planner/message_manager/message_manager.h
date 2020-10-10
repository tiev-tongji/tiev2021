#ifndef _MESSAGE_MANAGER_
#define _MESSAGE_MANAGER_

#include "nature/timestamp.h"
#include "nature.h"
#include <zcm/zcm-cpp.hpp>
#include <thread>
#include <vector>
#include <tuple>
#include <thread>
#include "mutexs/shared_mutex.h"
#include "nature/timestamp.h"
#include "nature/point.h"
#include "tievmsg.h"
#include "config/config.h"
using namespace std;

namespace TiEV{

class NavInfo{
public:
	bool detected = false;
    Point current_position;

    double current_speed;
	bool reliable = false;
};

class SlamInfo{
	public:
		bool detected = false;
		double x;
		double y;
		double heading;
		bool reliable =false;
};

class LidarMap{
public:
	bool detected = false;
    Point current_position;

    //map[i][j] represents a point at ith row and jth column
    //map is the lidar message detected when car is at current_position
    unsigned char map[GRID_ROW][GRID_COL];
};

class DynamicObj{
public:
	int id = -1;
	ObjectType type = UNKNOWN;
	double width = 0;
	double length = 0;
	double heading = 0;
	vector<Point> path;
	vector<Point> corners;
};

class DynamicObjList{
public:
	bool detected = false;
	vector<DynamicObj> dynamic_obj_list;
};

class WarningObj{
public:
	int id;
	int type;
	double x;
	double y;
};

class WarningObjList{
public:
	bool detected = false;
	vector<WarningObj> warning_obj_list;
};

class TrafficLight{
public:
	bool detected = false;
	bool left = true;
	bool straight = true;
	bool right = true;
};

class Pedestrian{
public:
	bool detected = false;
	vector<Point> positions;
};

class ParkingLot{
public:
	Point left_back;
	Point right_back;
	Point left_front;
	Point right_front;
};

class ParkingLotList{
public:
	bool detected = false;
	vector<ParkingLot> parking_lot_list;
};

class LaneLine{
public:
	LineType type;
	double distance;
	vector<Point> points;
};

class Lane{
public:
	int type;
	double width;
	Point stop_point;
	LaneLine left_line;
	LaneLine right_line;
};

class LaneList{
public:
	bool detected = false;
	int current_id;
	vector<Lane> lane_list;
};

class SecurityInfo{
public:
    SecurityInfo(){
        throw "unimplemented class";
    }
};

static mutex msg_manager_mtx;
class MessageManager{

public:

	static MessageManager* getInstance(){
		msg_manager_mtx.lock();
		static MessageManager instance;
		msg_manager_mtx.unlock();
		return &instance;
	}

    const static std::time_t NAV_INFO_TIMEOUT_US = 1e6;
    const static std::time_t LIDAR_MAP_TIMEOUT_US = 1e6;
    const static std::time_t LANE_TIMEOUT_US = 2e5;
    const static std::time_t TRAFFIC_LIGHT_TIMEOUT_US = 1e6;
    const static std::time_t PARKING_SLOT_TIMEOUT_US = 1e6;
    const static std::time_t OBJECT_LIST_TIMEOUT_US = 1e6;
	const static std::time_t SLAM_LOC_TIMEOUT_US = 1e6;
	const static std::time_t CAN_INFO_TIMEOUT_US = 1e5;
    const static int OBJECTS_SOURCE_NUM = 3;

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

private:
	MessageManager(){};
	~MessageManager(){};

    class Handler{
    public:
        void handleNAVINFO(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structNAVINFO *msg);
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

        structNAVINFO tmp_nav;
        structFUSIONMAP tmp_map;
        structOBJECTLIST tmp_objects[OBJECTS_SOURCE_NUM];
        structTRAFFICLIGHT tmp_traffic;
		structSLAMLOC tmp_slam_loc;
		structCANINFO tmp_can_info;
        structLANES tmp_lanes;
        structPARKINGSLOTS tmp_slot;
    };

    Handler inner_handler;
	zcm::ZCM zcm_ipc{"ipc"};
	zcm::ZCM zcm_udp{""};
};

}

#endif
