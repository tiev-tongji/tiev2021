#include "message_manager.h"

#include <iostream>
#include <sstream>
#include <string>

bool MessageManager::getNavInfo(NavInfo& nav_info) {
    std::shared_lock<std::shared_mutex> lck(inner_handler.nav_mtx);
    time_t                              current_time = getTimeStamp();

    nav_info.detected      = false;
    nav_info.reliable      = false;
    nav_info.car_pose      = Pose(CAR_CEN_ROW, CAR_CEN_COL, PI);
    nav_info.current_speed = 0;
    nav_info.lon           = 0;
    nav_info.lat           = 0;
    if(current_time - inner_handler.update_time_nav_info
       < NAV_INFO_TIMEOUT_US) {
        nav_info.detected                      = true;
        nav_info.car_pose.utm_position.utm_x   = inner_handler.tmp_nav.utmX;
        nav_info.car_pose.utm_position.utm_y   = inner_handler.tmp_nav.utmY;
        nav_info.car_pose.utm_position.heading = inner_handler.tmp_nav.mHeading;
        nav_info.current_speed                 = inner_handler.tmp_nav.mSpeed3d;
        nav_info.lon                           = inner_handler.tmp_nav.mLon;
        nav_info.lat                           = inner_handler.tmp_nav.mLat;
        if(inner_handler.tmp_nav.mRTKStatus == 1
           || inner_handler.tmp_nav.isReckoningVaild) {
            nav_info.reliable = true;
        }
    }

    if(current_time - inner_handler.update_time_can_info < CAN_INFO_TIMEOUT_US)
        nav_info.current_speed = inner_handler.tmp_can_info.carspeed / 360.0;
    return nav_info.detected;
}

bool MessageManager::getMap(LidarMap& lidar_map) {
    std::shared_lock<std::shared_mutex> lck(inner_handler.map_mtx);
    time_t                              current_time = getTimeStamp();

    lidar_map.detected = false;
    memset(lidar_map.map, 0x00, sizeof(lidar_map.map));
    if(current_time - inner_handler.update_time_fusion_map
       < LIDAR_MAP_TIMEOUT_US) {
        lidar_map.detected = true;
        memcpy(lidar_map.map, inner_handler.tmp_map.cells,
               sizeof(lidar_map.map));
    }
    return lidar_map.detected;
}

bool MessageManager::getRainSignal(RainSignal& rain_signal) {
    inner_handler.rain_mtx.lock_shared();
    time_t current_time = getTimeStamp();

    rain_signal.detected = false;
    if(current_time - inner_handler.update_time_rain_signal
       < RAIN_SIGNAL_TIMEOUT_US) {
        rain_signal.detected = true;
        rain_signal.signal   = inner_handler.tmp_rain_signal.rain_signal;
    }
    inner_handler.rain_mtx.unlock_shared();
    return rain_signal.detected;
}

bool MessageManager::getDynamicObjList(DynamicObjList& dynamic_obj_list) {
    inner_handler.objects_mtx.lock_shared();
    time_t current_time_us = getTimeStamp();
    dynamic_obj_list.dynamic_obj_list.clear();
    dynamic_obj_list.detected = false;
    constexpr double y_offset = 0;
    for(int i = 0; i < MessageManager::OBJECTS_SOURCE_NUM; ++i)
        if(current_time_us - inner_handler.update_time_objects[i]
           < OBJECT_LIST_TIMEOUT_US) {
            if(!inner_handler.tmp_objects[i].obj.size()) continue;
            dynamic_obj_list.detected = true;
            int old_size = dynamic_obj_list.dynamic_obj_list.size();
            dynamic_obj_list.dynamic_obj_list.resize(
                old_size + inner_handler.tmp_objects[i].obj.size());
            for(int j = 0; j < inner_handler.tmp_objects[i].obj.size(); ++j) {
                OBJECT&     obj = inner_handler.tmp_objects[i].obj[j];
                DynamicObj& new_obj =
                    dynamic_obj_list.dynamic_obj_list[j + old_size];

                new_obj.id      = obj.id;
                new_obj.type    = (DynamicType)obj.obj_type;
                new_obj.width   = obj.width;
                new_obj.length  = obj.length;
                new_obj.heading = obj.theta + PI / 2;
                normalizeAngle(new_obj.heading);
                new_obj.corners.resize(4);
                new_obj.v = obj.v;
#define copy_point(a, b)                                      \
    a.x = CAR_CEN_ROW - ((b.y + y_offset) / GRID_RESOLUTION); \
    a.y = CAR_CEN_COL + b.x / GRID_RESOLUTION;
                copy_point(new_obj.corners[0], obj.corners.p1);
                copy_point(new_obj.corners[1], obj.corners.p2);
                copy_point(new_obj.corners[2], obj.corners.p3);
                copy_point(new_obj.corners[3], obj.corners.p4);
                new_obj.path.resize(obj.path.size());
                for(int k = 0; k < obj.path.size(); ++k) {
                    copy_point(new_obj.path[k], obj.path[k]);
                    new_obj.path[k].x =
                        CAR_CEN_ROW
                        - ((obj.path[k].y + y_offset) / GRID_RESOLUTION);
                    new_obj.path[k].y =
                        CAR_CEN_COL + obj.path[k].x / GRID_RESOLUTION;
                    new_obj.path[k].t   = k;
                    new_obj.path[k].ang = new_obj.heading;
                    new_obj.path[k].v   = obj.v;
                }
#undef copy_point
            }
        }

    inner_handler.objects_mtx.unlock_shared();
    return dynamic_obj_list.detected;
}

bool MessageManager::getWarningObjList(WarningObjList& warning_obj_list) {
    return false;
}

bool MessageManager::getTrafficLight(TrafficLight& traffic_light) {
    inner_handler.traffic_mtx.lock_shared();
    time_t current_time    = getTimeStamp();
    traffic_light.detected = false;
    if(current_time - inner_handler.update_time_traffic_light
       < TRAFFIC_LIGHT_TIMEOUT_US) {
        traffic_light.detected = true;
        traffic_light.left     = inner_handler.tmp_traffic.left;
        traffic_light.right    = inner_handler.tmp_traffic.right;
        traffic_light.straight = inner_handler.tmp_traffic.forward;
    }
    inner_handler.traffic_mtx.unlock_shared();
    return traffic_light.detected;
}

bool MessageManager::getLaneList(LaneList& lane_list) {
    inner_handler.lane_mtx.lock_shared();
    lane_list.detected = false;
    lane_list.lane_list.clear();
    time_t current_time = getTimeStamp();
    if(current_time - inner_handler.update_time_lane < LANE_TIMEOUT_US) {
        if(inner_handler.tmp_lanes.num > 0) {
            lane_list.detected   = true;
            lane_list.current_id = inner_handler.tmp_lanes.current_lane_id;
            for(int i = 0; i < inner_handler.tmp_lanes.num; ++i) {
                TiEVLane lane;
                lane.type = 0x00;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x01)
                    lane.type |= RoadDirection::STRAIGHT;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x02)
                    lane.type |= RoadDirection::LEFT;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x04)
                    lane.type |= RoadDirection::RIGHT;
                // cout << "lane" << i << " direction:" << lane.type << " visual
                // direction:" << inner_handler.tmp_lanes.lanes[i].lane_type <<
                // endl; if(lane.type == 0x00) lane.type = 0x07;
                lane.width = inner_handler.tmp_lanes.lanes[i].width;
                switch(inner_handler.tmp_lanes.lanes[i].left_line.line_type) {
                case 1:
                    lane.left_line.type = LineType::DASH;
                    break;
                case 3:
                    lane.left_line.type = LineType::DASH;
                    break;
                case 0:
                    lane.left_line.type = LineType::SOLID;
                    break;
                case 2:
                    lane.left_line.type = LineType::SOLID;
                    break;
                default:
                    lane.left_line.type = LineType::UNKNOWN_LINE;
                }
                if(inner_handler.tmp_lanes.lanes[i]
                       .left_line.boundary_confidence
                   == 2)
                    lane.left_line.type = LineType::BOUNDARY;
                if(i < lane_list.current_id)
                    lane.left_line.distance =
                        -inner_handler.tmp_lanes.lanes[i].left_line.distance;
                else
                    lane.left_line.distance =
                        inner_handler.tmp_lanes.lanes[i].left_line.distance;
                switch(inner_handler.tmp_lanes.lanes[i].right_line.line_type) {
                case 1:
                    lane.right_line.type = LineType::DASH;
                    break;
                case 3:
                    lane.right_line.type = LineType::DASH;
                    break;
                case 0:
                    lane.right_line.type = LineType::SOLID;
                    break;
                case 2:
                    lane.right_line.type = LineType::SOLID;
                    break;
                default:
                    lane.right_line.type = LineType::UNKNOWN_LINE;
                }
                if(inner_handler.tmp_lanes.lanes[i]
                       .right_line.boundary_confidence
                   == 2)
                    lane.left_line.type = LineType::BOUNDARY;
                if(i <= lane_list.current_id)
                    lane.right_line.distance =
                        -inner_handler.tmp_lanes.lanes[i].right_line.distance;
                else
                    lane.right_line.distance =
                        inner_handler.tmp_lanes.lanes[i].right_line.distance;
                for(auto& point :
                    inner_handler.tmp_lanes.lanes[i].left_line.points) {
                    TiEVLinePoint p;
                    p.type = lane.left_line.type;
                    p.x    = CAR_CEN_ROW - (point.x / GRID_RESOLUTION);
                    p.y    = CAR_CEN_COL - (point.y / GRID_RESOLUTION);
                    if(!p.in_map()) continue;
                    lane.left_line.points.push_back(p);
                }
                for(auto& point :
                    inner_handler.tmp_lanes.lanes[i].right_line.points) {
                    TiEVLinePoint p;
                    p.type = lane.right_line.type;
                    p.x    = CAR_CEN_ROW - (point.x / GRID_RESOLUTION);
                    p.y    = CAR_CEN_COL - (point.y / GRID_RESOLUTION);
                    if(!p.in_map()) continue;
                    lane.right_line.points.push_back(p);
                }
                lane_list.lane_list.push_back(lane);
            }
            inner_handler.lane_mtx.unlock();
        }
    }
    inner_handler.lane_mtx.unlock_shared();
    return lane_list.detected;
}

bool MessageManager::getParkingLotList(ParkingLotList& parking_lot_list) {
    inner_handler.parking_slot_mtx.lock_shared();
    time_t current_time       = getTimeStamp();
    parking_lot_list.detected = false;
    if(current_time - inner_handler.update_time_parking_slots
           < PARKING_SLOT_TIMEOUT_US
       && inner_handler.tmp_slot.num) {
        parking_lot_list.detected = true;
        parking_lot_list.parking_lot_list.resize(inner_handler.tmp_slot.num);
        for(int i = 0; i < inner_handler.tmp_slot.num; ++i) {
            parking_lot_list.parking_lot_list[i].right_front.x =
                CAR_CEN_ROW
                - inner_handler.tmp_slot.parking_slots[i].front_right.y
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_front.y =
                CAR_CEN_COL
                + inner_handler.tmp_slot.parking_slots[i].front_right.x
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_front.x =
                CAR_CEN_ROW
                - inner_handler.tmp_slot.parking_slots[i].front_left.y
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_front.y =
                CAR_CEN_COL
                + inner_handler.tmp_slot.parking_slots[i].front_left.x
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_back.x =
                CAR_CEN_ROW
                - inner_handler.tmp_slot.parking_slots[i].rear_left.y
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_back.y =
                CAR_CEN_COL
                + inner_handler.tmp_slot.parking_slots[i].rear_left.x
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_back.x =
                CAR_CEN_ROW
                - inner_handler.tmp_slot.parking_slots[i].rear_right.y
                      / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_back.y =
                CAR_CEN_COL
                + inner_handler.tmp_slot.parking_slots[i].rear_right.x
                      / GRID_RESOLUTION;
        }
    }
    inner_handler.parking_slot_mtx.unlock_shared();
    return parking_lot_list.detected;
}

bool MessageManager::getPedestrian(Pedestrian& pedestrian) {
    // TODO
    return false;
}

bool MessageManager::getSlamInfo(SlamInfo& slam_info) {
    inner_handler.slam_loc_mtx.lock_shared();
    slam_info.detected = getTimeStamp() - inner_handler.update_time_slam_loc
                         < SLAM_LOC_TIMEOUT_US;
    if(slam_info.detected) {
        slam_info.x        = inner_handler.tmp_slam_loc.x;
        slam_info.y        = inner_handler.tmp_slam_loc.y;
        slam_info.heading  = inner_handler.tmp_slam_loc.mHeading;
        slam_info.reliable = inner_handler.tmp_slam_loc.SLAMStatus;
    }
    else
        slam_info.reliable = false;
    inner_handler.slam_loc_mtx.unlock_shared();
    return slam_info.detected;
}

void MessageManager::Handler::handleNAVINFO(const zcm::ReceiveBuffer* rbuf,
                                            const std::string&        chan,
                                            const structNAVINFO*      msg) {
    std::unique_lock<std::shared_mutex> lock(nav_mtx);
    tmp_nav              = *msg;
    update_time_nav_info = getTimeStamp();
}

void MessageManager::Handler::handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf,
                                              const std::string&        chan,
                                              const structFUSIONMAP*    msg) {
    std::unique_lock<std::shared_mutex> lock(map_mtx);
    tmp_map                = *msg;
    update_time_fusion_map = getTimeStamp();
}

void MessageManager::Handler::handleTRAFFICLIGHT(
    const zcm::ReceiveBuffer* rbuf, const std::string& chan,
    const MsgTrafficLightSignal* msg) {
    std::unique_lock<std::shared_mutex> lock(traffic_mtx);
    tmp_traffic               = *msg;
    update_time_traffic_light = getTimeStamp();
}

void MessageManager::Handler::handleLANES(const zcm::ReceiveBuffer*    rbuf,
                                          const std::string&           chan,
                                          const structRoadMarkingList* msg) {
    std::unique_lock<std::shared_mutex> lock(lane_mtx);
    update_time_lane = getTimeStamp();
    tmp_lanes        = *msg;
}

void MessageManager::Handler::handlePARKINGSLOTS(
    const zcm::ReceiveBuffer* rbuf, const std::string& chan,
    const structPARKINGSLOTS* msg) {
    std::unique_lock<std::shared_mutex> lock(parking_slot_mtx);
    tmp_slot                  = *msg;
    update_time_parking_slots = getTimeStamp();
}

void MessageManager::Handler::handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf,
                                               const std::string&        chan,
                                               const structOBJECTLIST*   msg) {
    std::unique_lock<std::shared_mutex> lock(objects_mtx);
    tmp_objects[msg->data_source]         = *msg;
    update_time_objects[msg->data_source] = getTimeStamp();
}

void MessageManager::Handler::handleSLAMLOC(const zcm::ReceiveBuffer* rbuf,
                                            const std::string&        chan,
                                            const structSLAMLOC*      msg) {
    std::unique_lock<std::shared_mutex> lock(slam_loc_mtx);
    tmp_slam_loc         = *msg;
    update_time_slam_loc = getTimeStamp();
}

void MessageManager::Handler::handleCANINFO(const zcm::ReceiveBuffer* rbuf,
                                            const std::string&        chan,
                                            const structCANINFO*      msg) {
    std::unique_lock<std::shared_mutex> lock(nav_mtx);
    tmp_can_info         = *msg;
    update_time_can_info = getTimeStamp();
}

void MessageManager::publishRemoteControl(
    const structREMOTECONTROL& remote_control) {
    zcm_udp.publish("REMOTECONTROL", &remote_control);
}

void MessageManager::publishSlamControl(const structSLAMCONTROL& slam_control) {
    zcm_udp.publish("SLAMCONTROL", &slam_control);
}

void MessageManager::msgReceiveIpc() {
    if(!zcm_ipc.good()) return;

    // zcm_ipc.subscribe("TRAFFICLIGHT", &Handler::handleTRAFFICLIGHT,
    // &inner_handler); zcm_ipc.run();
}

void MessageManager::msgReceiveUdp() {
    if(!zcm_udp.good()) return;

    zcm_udp.subscribe("msgTrafficLightSignal", &Handler::handleTRAFFICLIGHT,
                      &inner_handler);
    zcm_udp.subscribe("NAVINFO", &Handler::handleNAVINFO, &inner_handler);
    zcm_udp.subscribe("FUSIONMAP", &Handler::handleFUSIONMAP, &inner_handler);
    zcm_udp.subscribe("OBJECTLIST", &Handler::handleOBJECTLIST, &inner_handler);
    zcm_udp.subscribe("PARKINGSLOTS", &Handler::handlePARKINGSLOTS,
                      &inner_handler);
    zcm_udp.subscribe("LANE_info", &Handler::handleLANES, &inner_handler);
    zcm_udp.subscribe("SLAMLOC", &Handler::handleSLAMLOC, &inner_handler);
    zcm_udp.subscribe("CANINFO", &Handler::handleCANINFO, &inner_handler);
    zcm_udp.run();
}
