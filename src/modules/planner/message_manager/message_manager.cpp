#include "message_manager.h"
#include <string>
#include <sstream>
#include <iostream>
#include "utils/utils.h"

using namespace std;

namespace TiEV{

bool MessageManager::getNavInfo(NavInfo& nav_info){
    inner_handler.nav_mtx.lock_shared();
    time_t current_time = getTimeStamp();

    nav_info.detected = false;
	nav_info.reliable = false;
    nav_info.current_position.lat = -1;
    nav_info.current_position.lon = -1;
    nav_info.current_position.utmX = 0;
    nav_info.current_position.utmY = 0;
    nav_info.current_position.x = CAR_CEN_ROW;
    nav_info.current_position.y = CAR_CEN_COL;
    nav_info.current_position.heading.setByRad(0);
    nav_info.current_speed = 0;
    if(current_time - inner_handler.update_time_nav_info < NAV_INFO_TIMEOUT_US){
        nav_info.detected = true;
        nav_info.current_position.lat = inner_handler.tmp_nav.mLat;
        nav_info.current_position.lon = inner_handler.tmp_nav.mLon;
        nav_info.current_position.utmX = inner_handler.tmp_nav.utmX;
        nav_info.current_position.utmY = inner_handler.tmp_nav.utmY;
        nav_info.current_position.x = CAR_CEN_ROW;
        nav_info.current_position.y = CAR_CEN_COL;
        nav_info.current_position.heading.setByRad(inner_handler.tmp_nav.mHeading);
        nav_info.current_speed = inner_handler.tmp_nav.mSpeed3d;
		if(inner_handler.tmp_nav.mRTKStatus == 1 || inner_handler.tmp_nav.isReckoningVaild) nav_info.reliable = true;
    }

    if(current_time - inner_handler.update_time_can_info < CAN_INFO_TIMEOUT_US)
        nav_info.current_speed = inner_handler.tmp_can_info.carspeed / 360.0;
    inner_handler.nav_mtx.unlock_shared();
    return nav_info.detected;
}

bool MessageManager::getMap(LidarMap& lidar_map){
    inner_handler.map_mtx.lock_shared();
    time_t current_time = getTimeStamp();

    lidar_map.detected = false;
    memset(lidar_map.map, 0x00, sizeof(lidar_map.map));
    if(current_time - inner_handler.update_time_fusion_map < LIDAR_MAP_TIMEOUT_US){
        lidar_map.detected = true;
        lidar_map.current_position = TiEV::Point::fromUTMXY(inner_handler.tmp_map.utmX, inner_handler.tmp_map.utmY);
        lidar_map.current_position.heading.setByRad(inner_handler.tmp_map.mHeading);

        memcpy(lidar_map.map, inner_handler.tmp_map.cells, sizeof(lidar_map.map));
		/*
		for(int r = 0; r < 401; r++){
			cout << "row" << r << ":";
			for(int c = 0; c < 151; c++){
				cout << " " << (int)lidar_map.map[r][c];
			}
			cout << endl;
		}
		*/
    }

    inner_handler.map_mtx.unlock_shared();
    return lidar_map.detected;
}

bool MessageManager::getDynamicObjList(DynamicObjList& dynamic_obj_list){
    inner_handler.objects_mtx.lock_shared();
    time_t current_time_us = getTimeStamp();
    dynamic_obj_list.dynamic_obj_list.clear();
    dynamic_obj_list.detected = false;
    for (int i = 0; i < MessageManager::OBJECTS_SOURCE_NUM; ++i)
        if(current_time_us - inner_handler.update_time_objects[i] < OBJECT_LIST_TIMEOUT_US){
            if(!inner_handler.tmp_objects[i].obj.size()) continue;
            dynamic_obj_list.detected = true;
            int old_size = dynamic_obj_list.dynamic_obj_list.size();
            dynamic_obj_list.dynamic_obj_list.resize(old_size +
                inner_handler.tmp_objects[i].obj.size());
            for(int j = 0; j < inner_handler.tmp_objects[i].obj.size(); ++j){
                OBJECT& obj = inner_handler.tmp_objects[i].obj[j];
                DynamicObj& new_obj = dynamic_obj_list.dynamic_obj_list[j + old_size];

                new_obj.id = obj.id;
                new_obj.type = (ObjectType)obj.obj_type;
                new_obj.width = obj.width;
                new_obj.length = obj.length;
                new_obj.heading = Angle::PI/2 - obj.theta;
                utils::normalizeAngle(new_obj.heading);
                new_obj.corners.resize(4);
                #define copy_point(a, b) a.x = CAR_CEN_ROW - ((b.y - 1.48) / GRID_RESOLUTION); \
				a.y = CAR_CEN_COL + b.x / GRID_RESOLUTION; a.v = obj.v
                copy_point(new_obj.corners[0], obj.corners.p1);
                copy_point(new_obj.corners[1], obj.corners.p2);
                copy_point(new_obj.corners[2], obj.corners.p3);
                copy_point(new_obj.corners[3], obj.corners.p4);
                cout << "obj" << j << "corner ru rb lb lu x y:" << " (" << obj.corners.p1.x << " " << obj.corners.p1.y << ") " << " (" << obj.corners.p2.x << " " << obj.corners.p2.y << ") "<< " (" << obj.corners.p3.x << " " << obj.corners.p3.y << ") "<< " (" << obj.corners.p4.x << " " << obj.corners.p4.y << ") "<< endl;
                new_obj.path.resize(obj.path.size());
                for(int k = 0; k < obj.path.size(); ++k){
                    copy_point(new_obj.path[k], obj.path[k]);
					new_obj.path[k].t = k;
                    new_obj.path[k].angle.setByRad(new_obj.heading);
                    cout << "obj" << j << "x y:" << obj.path[k].x << " " << obj.path[k].y << endl;
                }
                #undef copy_point
            }
        }

    inner_handler.objects_mtx.unlock_shared();
    return dynamic_obj_list.detected;
}

bool MessageManager::getWarningObjList(WarningObjList& warning_obj_list){
    return false;
}

bool MessageManager::getTrafficLight(TrafficLight& traffic_light){
    inner_handler.traffic_mtx.lock_shared();
    time_t current_time = getTimeStamp();
    traffic_light.detected = false;
    if(current_time - inner_handler.update_time_traffic_light < TRAFFIC_LIGHT_TIMEOUT_US){
        traffic_light.detected = true;
        uint8_t raw_signal = inner_handler.tmp_traffic.raw_signal;
        uint8_t turn_signal = inner_handler.tmp_traffic.turn_signal;
        traffic_light.left = inner_handler.tmp_traffic.turn_signal & 0x4;
        traffic_light.right = inner_handler.tmp_traffic.turn_signal & 0x1;
        traffic_light.straight = inner_handler.tmp_traffic.turn_signal & 0x2;
    }
    inner_handler.traffic_mtx.unlock_shared();
    return traffic_light.detected;
}

bool MessageManager::getLaneList(LaneList& lane_list){
    inner_handler.lane_mtx.lock_shared();
    lane_list.detected = false;
    lane_list.lane_list.clear();
    time_t current_time = getTimeStamp();
    if(current_time - inner_handler.update_time_lane < LANE_TIMEOUT_US){
        if(inner_handler.tmp_lanes.num > 0){
			lane_list.detected = true;
			lane_list.current_id = inner_handler.tmp_lanes.current_lane_id;
            for(int i = 0; i < inner_handler.tmp_lanes.num; ++i){
				Lane lane;
                lane.type = 0x00;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x01)
                    lane.type |= RoadDirection::STRAIGHT;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x02)
                    lane.type |= RoadDirection::LEFT;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x04)
                    lane.type |= RoadDirection::RIGHT;
                    //cout << "lane" << i << " direction:" << lane.type << " visual direction:" << inner_handler.tmp_lanes.lanes[i].lane_type << endl;
                //if(lane.type == 0x00) lane.type = 0x07;
				lane.width = inner_handler.tmp_lanes.lanes[i].width;
				if(inner_handler.tmp_lanes.lanes[i].left_line.line_type & 0x01)
                    lane.left_line.type = LineType::DASHED;
				else lane.left_line.type = LineType::SOLID;
				lane.left_line.distance = -inner_handler.tmp_lanes.lanes[i].left_line.distance;
				if(inner_handler.tmp_lanes.lanes[i].right_line.line_type & 0x01)
                    lane.right_line.type = LineType::DASHED;
				else lane.right_line.type = LineType::SOLID;
				lane.right_line.distance = -inner_handler.tmp_lanes.lanes[i].right_line.distance;
				for(auto &point: inner_handler.tmp_lanes.lanes[i].left_line.points){
					Point p;
					p.x = CAR_CEN_ROW - (point.y / GRID_RESOLUTION);
					p.y = CAR_CEN_COL + (point.x / GRID_RESOLUTION);
					if(p.x < 0 || p.x >= GRID_ROW || p.y < 0 || p.y >= GRID_COL) continue;
					lane.left_line.points.push_back(p);
				}
				for(auto &point: inner_handler.tmp_lanes.lanes[i].right_line.points){
					Point p;
					p.x = CAR_CEN_ROW - (point.y / GRID_RESOLUTION);
					p.y = CAR_CEN_COL + (point.x / GRID_RESOLUTION);
					if(p.x < 0 || p.x >= GRID_ROW || p.y < 0 || p.y >= GRID_COL) continue;
					lane.right_line.points.push_back(p);
				}
				if(inner_handler.tmp_lanes.lanes[i].stop_point.y != -1){
					lane.stop_point.x = CAR_CEN_ROW - (inner_handler.tmp_lanes.lanes[i].stop_point.y / GRID_RESOLUTION);
					lane.stop_point.y = CAR_CEN_COL + (inner_handler.tmp_lanes.lanes[i].stop_point.x / GRID_RESOLUTION);
				}
				else{
					lane.stop_point.x = -1;
					lane.stop_point.y = -1;
				}
				lane_list.lane_list.push_back(lane);
            }
            inner_handler.lane_mtx.unlock();
        }
    }
    inner_handler.lane_mtx.unlock_shared();
    return lane_list.detected;
}

bool MessageManager::getParkingLotList(ParkingLotList& parking_lot_list){
    inner_handler.parking_slot_mtx.lock_shared();
    time_t current_time = getTimeStamp();
    parking_lot_list.detected = false;
    if(current_time - inner_handler.update_time_parking_slots < PARKING_SLOT_TIMEOUT_US
        && inner_handler.tmp_slot.num){
        parking_lot_list.detected = true;
        parking_lot_list.parking_lot_list.resize(inner_handler.tmp_slot.num);
        for(int i = 0; i < inner_handler.tmp_slot.num; ++i){
            parking_lot_list.parking_lot_list[i].right_front.x = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].front_right.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_front.y = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].front_right.x / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_front.x = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].front_left.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_front.y = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].front_left.x / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_back.x = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].rear_left.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_back.y = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].rear_left.x / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_back.x = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].rear_right.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_back.y = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].rear_right.x / GRID_RESOLUTION;
        }
    }
    inner_handler.parking_slot_mtx.unlock_shared();
    return parking_lot_list.detected;
}

bool MessageManager::getPedestrian(Pedestrian& pedestrian){
	//TODO
}

bool MessageManager::getSlamInfo(SlamInfo& slam_info){
    inner_handler.slam_loc_mtx.lock_shared();
    slam_info.detected = getTimeStamp() - inner_handler.update_time_slam_loc
        < SLAM_LOC_TIMEOUT_US;
    if(slam_info.detected){
        slam_info.x = inner_handler.tmp_slam_loc.x;
        slam_info.y = inner_handler.tmp_slam_loc.y;
        slam_info.heading = inner_handler.tmp_slam_loc.mHeading;
        slam_info.reliable = inner_handler.tmp_slam_loc.SLAMStatus;
    }
    else slam_info.reliable = false;
    inner_handler.slam_loc_mtx.unlock_shared();
    return slam_info.detected;
}

void MessageManager::Handler::handleNAVINFO(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structNAVINFO *msg){
    nav_mtx.lock();
    tmp_nav = *msg;
    update_time_nav_info = getTimeStamp();
    nav_mtx.unlock();
}

void MessageManager::Handler::handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structFUSIONMAP* msg){
    map_mtx.lock();
    tmp_map = * msg;
    update_time_fusion_map = getTimeStamp();
    map_mtx.unlock();
}

void MessageManager::Handler::handleTRAFFICLIGHT(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structTRAFFICLIGHT* msg){
    traffic_mtx.lock();
    tmp_traffic = *msg;
    update_time_traffic_light = getTimeStamp();
    traffic_mtx.unlock();
}

void MessageManager::Handler::handleLANES(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structLANES* msg){
    lane_mtx.lock();
    update_time_lane = getTimeStamp();
    tmp_lanes = * msg;
    lane_mtx.unlock();
}

void MessageManager::Handler::handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structPARKINGSLOTS*  msg){
    parking_slot_mtx.lock();
    tmp_slot = * msg;
    update_time_parking_slots = getTimeStamp();
    parking_slot_mtx.unlock();
}

void MessageManager::Handler::handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structOBJECTLIST* msg){
    objects_mtx.lock();
    tmp_objects[msg->data_source] = *msg;
    update_time_objects[msg->data_source] = getTimeStamp();
    objects_mtx.unlock();
}

void MessageManager::Handler::handleSLAMLOC(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structSLAMLOC* msg){
    slam_loc_mtx.lock();
    tmp_slam_loc = *msg;
    update_time_slam_loc = getTimeStamp();
    slam_loc_mtx.unlock();
}

void MessageManager::Handler::handleCANINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO* msg){
    nav_mtx.lock();
    tmp_can_info = *msg;
    update_time_can_info = getTimeStamp();
    nav_mtx.unlock();
}

void MessageManager::publishPath(const structAIMPATH& path){
    zcm_udp.publish("AIMPATH", &path);
}

void MessageManager::publishRemoteControl(const structREMOTECONTROL& remote_control){
	zcm_udp.publish("REMOTECONTROL", &remote_control);
}

void MessageManager::publishSlamControl(const structSLAMCONTROL& slam_control){
    zcm_udp.publish("SLAMCONTROL", &slam_control);
}

void MessageManager::msgReceiveIpc(){
    if(!zcm_ipc.good()) return;

    //zcm_ipc.subscribe("TRAFFICLIGHT", &Handler::handleTRAFFICLIGHT, &inner_handler);
	//zcm_ipc.run();
}

void MessageManager::msgReceiveUdp(){
    if(!zcm_udp.good()) return;

    zcm_udp.subscribe("TRAFFICLIGHT", &Handler::handleTRAFFICLIGHT, &inner_handler);
    zcm_udp.subscribe("NAVINFO", &Handler::handleNAVINFO, &inner_handler);
    zcm_udp.subscribe("FUSIONMAP", &Handler::handleFUSIONMAP, &inner_handler);
    zcm_udp.subscribe("OBJECTLIST", &Handler::handleOBJECTLIST, &inner_handler);
    zcm_udp.subscribe("PARKINGSLOTS", &Handler::handlePARKINGSLOTS, &inner_handler);
    zcm_udp.subscribe("LANE_info", &Handler::handleLANES, &inner_handler);
    zcm_udp.subscribe("SLAMLOC", &Handler::handleSLAMLOC, &inner_handler);
    zcm_udp.subscribe("CANINFO", &Handler::handleCANINFO, &inner_handler);
	zcm_udp.run();
}

}
