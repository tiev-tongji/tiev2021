#include "message_manager.h"
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace TiEV {

bool MessageManager::getNavInfo(NavInfo& nav_info) {
    inner_handler.nav_mtx.lock_shared();
    time_t current_time = getTimeStamp();

    nav_info.detected      = false;
    nav_info.reliable      = false;
    nav_info.car_pose      = Pose(CAR_CEN_ROW, CAR_CEN_COL, PI);
    nav_info.current_speed = 0;
    nav_info.lon           = 0;
    nav_info.lat           = 0;
    if(current_time - inner_handler.update_time_nav_info < NAV_INFO_TIMEOUT_US) {
        nav_info.detected                      = true;
        nav_info.car_pose.utm_position.utm_x   = inner_handler.tmp_nav.utmX;
        nav_info.car_pose.utm_position.utm_y   = inner_handler.tmp_nav.utmY;
        nav_info.car_pose.utm_position.heading = inner_handler.tmp_nav.mHeading;
        nav_info.current_speed                 = inner_handler.tmp_nav.mSpeed3d;
        nav_info.lon                           = inner_handler.tmp_nav.mLon;
        nav_info.lat                           = inner_handler.tmp_nav.mLat;
        if(inner_handler.tmp_nav.mRTKStatus == 1 || inner_handler.tmp_nav.isReckoningVaild) {
            nav_info.reliable = true;
        }
    }

    if(current_time - inner_handler.update_time_can_info < CAN_INFO_TIMEOUT_US) nav_info.current_speed = inner_handler.tmp_can_info.carspeed / 360.0;
    inner_handler.nav_mtx.unlock_shared();
    return nav_info.detected;
}

bool MessageManager::getMap(LidarMap& lidar_map) {
    inner_handler.map_mtx.lock_shared();
    time_t current_time = getTimeStamp();

    lidar_map.detected = false;
    memset(lidar_map.map, 0x00, sizeof(lidar_map.map));
    if(current_time - inner_handler.update_time_fusion_map < LIDAR_MAP_TIMEOUT_US) {
        lidar_map.detected = true;
        memcpy(lidar_map.map, inner_handler.tmp_map.cells, sizeof(lidar_map.map));
    }
    inner_handler.map_mtx.unlock_shared();
    return lidar_map.detected;
}

bool MessageManager::getRainSignal(RainSignal& rain_signal) {
    inner_handler.rain_mtx.lock_shared();
    time_t current_time = getTimeStamp();

    rain_signal.detected = false;
    if(current_time - inner_handler.update_time_rain_signal < RAIN_SIGNAL_TIMEOUT_US) {
        rain_signal.detected = true;
        rain_signal.signal   = inner_handler.tmp_rain_signal.rain_signal;
    }
    inner_handler.rain_mtx.unlock_shared();
    return rain_signal.detected;
}

void MessageManager::clearTextInfo() {
    text_info.clear();
}

void MessageManager::setTextInfo() {
    visualization.text_info.clear();
    visualization.text_info_size = 0;
    for(const auto& itr : text_info) {
        visText vis_text;
        vis_text.name  = itr.first;
        vis_text.value = itr.second;
        visualization.text_info.emplace_back(vis_text);
        visualization.text_info_size += 1;
    }
}

void MessageManager::addTextInfo(const string& name, const string& value) {
    text_info[name] = value;
}

bool MessageManager::getDynamicObjList(DynamicObjList& dynamic_obj_list) {
    inner_handler.objects_mtx.lock_shared();
    time_t current_time_us = getTimeStamp();
    dynamic_obj_list.dynamic_obj_list.clear();
    dynamic_obj_list.detected = false;
    for(int i = 0; i < MessageManager::OBJECTS_SOURCE_NUM; ++i)
        if(current_time_us - inner_handler.update_time_objects[i] < OBJECT_LIST_TIMEOUT_US) {
            if(!inner_handler.tmp_objects[i].obj.size()) continue;
            dynamic_obj_list.detected = true;
            int old_size              = dynamic_obj_list.dynamic_obj_list.size();
            dynamic_obj_list.dynamic_obj_list.resize(old_size + inner_handler.tmp_objects[i].obj.size());
            for(int j = 0; j < inner_handler.tmp_objects[i].obj.size(); ++j) {
                OBJECT&     obj     = inner_handler.tmp_objects[i].obj[j];
                DynamicObj& new_obj = dynamic_obj_list.dynamic_obj_list[j + old_size];

                new_obj.id      = obj.id;
                new_obj.type    = (ObjectType)obj.obj_type;
                new_obj.width   = obj.width;
                new_obj.length  = obj.length;
                new_obj.heading = PI / 2 - obj.theta;
                normalizeAngle(new_obj.heading);
                new_obj.corners.resize(4);
#define copy_point(a, b)                                  \
    a.x = CAR_CEN_ROW - ((b.y - 1.48) / GRID_RESOLUTION); \
    a.y = CAR_CEN_COL + b.x / GRID_RESOLUTION;
                copy_point(new_obj.corners[2], obj.corners.p3);
                copy_point(new_obj.corners[3], obj.corners.p4);
                /*
                cout << "obj" << j << "corner ru rb lb lu x y:"
                     << " (" << obj.corners.p1.x << " " << obj.corners.p1.y << ") "
                     << " (" << obj.corners.p2.x << " " << obj.corners.p2.y << ") "
                     << " (" << obj.corners.p3.x << " " << obj.corners.p3.y << ") "
                     << " (" << obj.corners.p4.x << " " << obj.corners.p4.y << ") " << endl;
                     */
                new_obj.path.resize(obj.path.size());
                for(int k = 0; k < obj.path.size(); ++k) {
                    copy_point(new_obj.path[k], obj.path[k]);
                    new_obj.path[k].x   = CAR_CEN_ROW - ((obj.path[k].y - 1.48) / GRID_RESOLUTION);
                    new_obj.path[k].y   = CAR_CEN_COL + obj.path[k].x / GRID_RESOLUTION;
                    new_obj.path[k].t   = k;
                    new_obj.path[k].ang = new_obj.heading;
                    // cout << "obj" << j << "x y:" << obj.path[k].x << " " << obj.path[k].y << endl;
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
    if(current_time - inner_handler.update_time_traffic_light < TRAFFIC_LIGHT_TIMEOUT_US) {
        traffic_light.detected = true;
        uint8_t raw_signal     = inner_handler.tmp_traffic.raw_signal;
        uint8_t turn_signal    = inner_handler.tmp_traffic.turn_signal;
        traffic_light.left     = inner_handler.tmp_traffic.turn_signal & 0x4;
        traffic_light.right    = inner_handler.tmp_traffic.turn_signal & 0x1;
        traffic_light.straight = inner_handler.tmp_traffic.turn_signal & 0x2;
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
                Lane lane;
                lane.type = 0x00;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x01) lane.type |= RoadDirection::STRAIGHT;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x02) lane.type |= RoadDirection::LEFT;
                if(inner_handler.tmp_lanes.lanes[i].lane_type & 0x04) lane.type |= RoadDirection::RIGHT;
                // cout << "lane" << i << " direction:" << lane.type << " visual direction:" << inner_handler.tmp_lanes.lanes[i].lane_type << endl;
                // if(lane.type == 0x00) lane.type = 0x07;
                lane.width = inner_handler.tmp_lanes.lanes[i].width;
                if(inner_handler.tmp_lanes.lanes[i].left_line.line_type & 0x01)
                    lane.left_line.type = LineType::DASH;
                else
                    lane.left_line.type = LineType::SOLID;
                lane.left_line.distance = -inner_handler.tmp_lanes.lanes[i].left_line.distance;
                if(inner_handler.tmp_lanes.lanes[i].right_line.line_type & 0x01)
                    lane.right_line.type = LineType::DASH;
                else
                    lane.right_line.type = LineType::SOLID;
                lane.right_line.distance = -inner_handler.tmp_lanes.lanes[i].right_line.distance;
                for(auto& point : inner_handler.tmp_lanes.lanes[i].left_line.points) {
                    LinePoint p;
                    p.type = lane.left_line.type;
                    p.x    = CAR_CEN_ROW - (point.y / GRID_RESOLUTION);
                    p.y    = CAR_CEN_COL + (point.x / GRID_RESOLUTION);
                    if(!p.in_map()) continue;
                    lane.left_line.points.push_back(p);
                }
                for(auto& point : inner_handler.tmp_lanes.lanes[i].right_line.points) {
                    LinePoint p;
                    p.type = lane.right_line.type;
                    p.x    = CAR_CEN_ROW - (point.y / GRID_RESOLUTION);
                    p.y    = CAR_CEN_COL + (point.x / GRID_RESOLUTION);
                    if(!p.in_map()) continue;
                    lane.right_line.points.push_back(p);
                }
                if(inner_handler.tmp_lanes.lanes[i].stop_point.y != -1) {
                    lane.stop_point.x = CAR_CEN_ROW - (inner_handler.tmp_lanes.lanes[i].stop_point.y / GRID_RESOLUTION);
                    lane.stop_point.y = CAR_CEN_COL + (inner_handler.tmp_lanes.lanes[i].stop_point.x / GRID_RESOLUTION);
                }
                else {
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

bool MessageManager::getParkingLotList(ParkingLotList& parking_lot_list) {
    inner_handler.parking_slot_mtx.lock_shared();
    time_t current_time       = getTimeStamp();
    parking_lot_list.detected = false;
    if(current_time - inner_handler.update_time_parking_slots < PARKING_SLOT_TIMEOUT_US && inner_handler.tmp_slot.num) {
        parking_lot_list.detected = true;
        parking_lot_list.parking_lot_list.resize(inner_handler.tmp_slot.num);
        for(int i = 0; i < inner_handler.tmp_slot.num; ++i) {
            parking_lot_list.parking_lot_list[i].right_front.x = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].front_right.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_front.y = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].front_right.x / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_front.x  = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].front_left.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_front.y  = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].front_left.x / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_back.x   = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].rear_left.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].left_back.y   = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].rear_left.x / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_back.x  = CAR_CEN_ROW - inner_handler.tmp_slot.parking_slots[i].rear_right.y / GRID_RESOLUTION;
            parking_lot_list.parking_lot_list[i].right_back.y  = CAR_CEN_COL + inner_handler.tmp_slot.parking_slots[i].rear_right.x / GRID_RESOLUTION;
        }
    }
    inner_handler.parking_slot_mtx.unlock_shared();
    return parking_lot_list.detected;
}

bool MessageManager::getPedestrian(Pedestrian& pedestrian) {
    // TODO
}

bool MessageManager::getSlamInfo(SlamInfo& slam_info) {
    inner_handler.slam_loc_mtx.lock_shared();
    slam_info.detected = getTimeStamp() - inner_handler.update_time_slam_loc < SLAM_LOC_TIMEOUT_US;
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

void MessageManager::Handler::handleNAVINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO* msg) {
    nav_mtx.lock();
    tmp_nav              = *msg;
    update_time_nav_info = getTimeStamp();
    nav_mtx.unlock();
}

void MessageManager::Handler::handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structFUSIONMAP* msg) {
    map_mtx.lock();
    tmp_map                = *msg;
    update_time_fusion_map = getTimeStamp();
    map_mtx.unlock();
}

void MessageManager::Handler::handleTRAFFICLIGHT(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structTRAFFICLIGHT* msg) {
    traffic_mtx.lock();
    tmp_traffic               = *msg;
    update_time_traffic_light = getTimeStamp();
    traffic_mtx.unlock();
}

void MessageManager::Handler::handleLANES(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structLANES* msg) {
    lane_mtx.lock();
    update_time_lane = getTimeStamp();
    tmp_lanes        = *msg;
    lane_mtx.unlock();
}

void MessageManager::Handler::handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structPARKINGSLOTS* msg) {
    parking_slot_mtx.lock();
    tmp_slot                  = *msg;
    update_time_parking_slots = getTimeStamp();
    parking_slot_mtx.unlock();
}

void MessageManager::Handler::handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structOBJECTLIST* msg) {
    objects_mtx.lock();
    tmp_objects[msg->data_source]         = *msg;
    update_time_objects[msg->data_source] = getTimeStamp();
    objects_mtx.unlock();
}

void MessageManager::Handler::handleSLAMLOC(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structSLAMLOC* msg) {
    slam_loc_mtx.lock();
    tmp_slam_loc         = *msg;
    update_time_slam_loc = getTimeStamp();
    slam_loc_mtx.unlock();
}

void MessageManager::Handler::handleCANINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO* msg) {
    nav_mtx.lock();
    tmp_can_info         = *msg;
    update_time_can_info = getTimeStamp();
    nav_mtx.unlock();
}

void MessageManager::publishPath(const structAIMPATH& path) {
    zcm_udp.publish("AIMPATH", &path);
}

void MessageManager::publishRemoteControl(const structREMOTECONTROL& remote_control) {
    zcm_udp.publish("REMOTECONTROL", &remote_control);
}

void MessageManager::publishSlamControl(const structSLAMCONTROL& slam_control) {
    zcm_udp.publish("SLAMCONTROL", &slam_control);
}

void MessageManager::publishVisualization() {
    zcm_udp.publish("VISUALIZATION", &visualization);
}

void MessageManager::msgReceiveIpc() {
    if(!zcm_ipc.good()) return;

    // zcm_ipc.subscribe("TRAFFICLIGHT", &Handler::handleTRAFFICLIGHT, &inner_handler);
    // zcm_ipc.run();
}

void MessageManager::msgReceiveUdp() {
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

void MessageManager::setTargets(const vector<Pose>& targets) {
    visualization.targets.clear();
    for(const auto& target : targets) {
        visPoint p;
        p.x = round(target.x);
        p.y = round(target.y);
        visualization.targets.emplace_back(p);
    }
    visualization.targets_size = visualization.targets.size();
}

void MessageManager::setStartPoint(const Pose& start_point) {
    visualization.start_point.x = start_point.x;
    visualization.start_point.y = start_point.y;
}

void MessageManager::setSafeMap(double safe_map[MAX_ROW][MAX_COL]) {
    memcpy(visualization.safe_map, safe_map, sizeof(visualization.safe_map));
}

void MessageManager::setPath(const vector<Pose>& path) {
    visPath p;
    for(int i = 0; i < path.size(); ++i) {
        visPoint point;
        point.x = path[i].x;
        point.y = path[i].y;
        p.path.emplace_back(point);
    }
    p.path_size = p.path.size();
    visualization.paths.emplace_back(p);
    visualization.paths_size += 1;
}

void MessageManager::setUsedMap(bool used_map[MAX_ROW][MAX_COL]) {
    memcpy(visualization.used_map, used_map, sizeof(visualization.used_map));
}

void MessageManager::clearPaths() {
    visualization.paths.clear();
    visualization.paths_size = 0;
}

void MessageManager::setSpeedPath(const SpeedPath& speed_path) {
    visualization.st_boundaries.clear();
    visualization.st_boundaries_size = 0;
    visualization.qp_speed_curve.clear();
    visualization.qp_speed_curve_size = 0;
    visualization.splines_speed_curve.clear();
    visualization.splines_speed_curve_size = 0;
    visualization.dp_speed.clear();
    visualization.dp_speed_size = 0;

    if(!speed_path.success || speed_path.path.empty()) return;

    for(int i = 0; i < speed_path.dp_speed_data.size(); ++i) {
        visSTPoint point;
        point.s = speed_path.dp_speed_data[i].s();
        point.t = speed_path.dp_speed_data[i].t();
        visualization.dp_speed.emplace_back(point);
    }
    visualization.dp_speed_size = visualization.dp_speed.size();

    for(int i = 0; i < speed_path.st_boundaries.size(); ++i) {
        visSTBoundary st_boundary;
        visSTPoint    ulp, urp, blp, brp;
        ulp.s           = speed_path.st_boundaries[i].upper_left_point().s();
        ulp.t           = speed_path.st_boundaries[i].upper_left_point().t();
        urp.s           = speed_path.st_boundaries[i].upper_right_point().s();
        urp.t           = speed_path.st_boundaries[i].upper_right_point().t();
        blp.s           = speed_path.st_boundaries[i].bottom_left_point().s();
        blp.t           = speed_path.st_boundaries[i].bottom_left_point().t();
        brp.s           = speed_path.st_boundaries[i].bottom_right_point().s();
        brp.t           = speed_path.st_boundaries[i].bottom_right_point().t();
        st_boundary.ulp = ulp;
        st_boundary.urp = urp;
        st_boundary.blp = blp;
        st_boundary.brp = brp;
        visualization.st_boundaries.emplace_back(st_boundary);
    }
    visualization.st_boundaries_size = visualization.st_boundaries.size();

    visualization.qp_or_splines = !speed_path.qp_success;

    for(auto spline : speed_path.splines.splines()) {
        visCoefficient coe;
        for(auto param : spline.spline_func().params()) {
            coe.params.emplace_back(param);
        }
        coe.params_num = coe.params.size();
        visualization.qp_speed_curve.emplace_back(coe);
    }
    visualization.qp_speed_curve_size = visualization.qp_speed_curve.size();

    for(const auto& spline : speed_path.cubic_splines) {
        visSpline2 tmp;
        visVec4f   xb, yb;
        xb.x   = spline.xb.x;
        xb.y   = spline.xb.y;
        xb.z   = spline.xb.z;
        xb.w   = spline.xb.w;
        yb.x   = spline.yb.x;
        yb.y   = spline.yb.y;
        yb.z   = spline.yb.z;
        yb.w   = spline.yb.w;
        tmp.xb = xb;
        tmp.yb = yb;
        visualization.splines_speed_curve.emplace_back(tmp);
    }
    visualization.splines_speed_curve_size = visualization.splines_speed_curve.size();
}
}