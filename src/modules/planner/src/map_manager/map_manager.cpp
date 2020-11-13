#include "map_manager.h"
#include "collision_check.h"
#include "tiev_utils.h"
#include <fstream>
#include <iostream>
#include <queue>

namespace TiEV {
using namespace std;

void MapManager::update() {
    MessageManager* message_manager = MessageManager::getInstance();
    message_manager->getNavInfo(map.nav_info);
    message_manager->getSlamInfo(map.slam_info);
    message_manager->getTrafficLight(map.traffic_light);
    message_manager->getDynamicObjList(map.dynamic_obj_list);
    message_manager->getParkingLotList(map.parking_lot_list);
    message_manager->getLaneList(map.lane_list);
    message_manager->getMap(map.lidar);
    //-------
    handleLidarMap();
    getLidarDisMap();
}

bool MapManager::requestGlobalPath(const NavInfo& nav_info) {
    cout << "request global path not implimented" << endl;
    return false;
}

void MapManager::readGlobalPathFile(const string& file_path) {
    const string error_header = "Read road-map file failed : ";

    global_path.clear();
    fstream input(file_path, ios::in);
    if(!input.is_open()) {
        cout << error_header << "file not exist." << endl;
        return;
    }

    string buffer, field;
    getline(input, buffer);
    stringstream buffer_stream(buffer);

    // check number of fields
    /*
    const int field_num = 12;
    int field_num_in_file = 0;
    */
    /*
    while(buffer_stream >> field){
        cout << "Add field: " << field << endl;
        ++field_num_in_file;
    }*/

    // if(field_num_in_file != field_num){
    //    cout << error_header << "content of input file is in illegal format." << endl;
    //    return;
    //}

    double lon, lat, utm_x, utm_y, heading, curve, lane_width;
    int    id, mode, speed_mode, event, block_type, lane_num, lane_seq;
    while(input >> id >> lon >> lat >> utm_x >> utm_y >> heading >> curve >> mode >> speed_mode >> event >> block_type >> lane_num >> lane_seq >> lane_width) {
        HDMapPoint point(utm_x, utm_y, heading, curve, (HDMapMode)mode, (HDMapSpeed)speed_mode, (HDMapEvent)event, (BlockType)block_type, lane_num, lane_seq, lane_width);
        global_path.push_back(point);
    }

    cout << "global path size: " << global_path.size() << endl;

    filtPoints();
    cout << "global path size after filtPoints " << global_path.size() << endl;
    setGlobalPathDirection();
}

void MapManager::filtPoints() {
    vector<HDMapPoint> tmp_points;
    tmp_points.reserve(global_path.size());
    double priv_ang = 0, cur_ang = 0, next_ang = 0;
    int    priv_lane = 0, cur_lane = 0, next_lane = 0;
    if(!global_path.empty()) tmp_points.push_back(global_path.front());

    for(int i = 1; i < global_path.size() - 1; ++i) {
        if(global_path[i].event != HDMapEvent::ENTRY_INTERSECTION && global_path[i].event != HDMapEvent::EXIT_INTERSECTION) {
            priv_ang         = global_path[i - 1].utm_position.heading;
            cur_ang          = global_path[i].utm_position.heading;
            next_ang         = global_path[i + 1].utm_position.heading;
            double delta_ang = priv_ang - next_ang;
            normalizeAngle(delta_ang);
            double std_ang = delta_ang / 2.0 + next_ang;
            normalizeAngle(std_ang);
            normalizeAngle(cur_ang);
            if(fabs(cur_ang - std_ang) > 0.6) continue;

            priv_lane = global_path[i - 1].lane_num;
            cur_lane  = global_path[i].lane_num;
            next_lane = global_path[i + 1].lane_num;
            if(priv_lane == next_lane && next_lane != cur_lane) continue;
        }

        tmp_points.push_back(global_path[i]);
    }

    if(global_path.size() > 1) tmp_points.push_back(global_path.back());

    swap(global_path, tmp_points);
}

void MapManager::setGlobalPathDirection() {
    RoadDirection current_direction = RoadDirection::STRAIGHT;
    double        heading_after = 0.0, heading_before = 0.0;
    int           exit_index, entry_index;
    for(int i = global_path.size(); i >= 0; i--) {
        if(global_path[i].event != HDMapEvent::ENTRY_INTERSECTION && global_path[i].event != HDMapEvent::EXIT_INTERSECTION) {
            global_path[i].direction = current_direction;
        }
        else if(global_path[i].event == HDMapEvent::EXIT_INTERSECTION) {
            heading_after = global_path[i].utm_position.heading;
            exit_index    = i;
        }
        else if(global_path[i].event == HDMapEvent::ENTRY_INTERSECTION) {
            heading_before       = global_path[i].utm_position.heading;
            entry_index          = i;
            double delta_heading = heading_after - heading_before;
            while(delta_heading < -PI)
                delta_heading += 2 * PI;
            while(delta_heading >= PI)
                delta_heading -= 2 * PI;
            if(delta_heading > PI / 3 && delta_heading < 2 * PI / 3) {
                current_direction = RoadDirection::LEFT;
            }
            else if(delta_heading < -PI / 3 && delta_heading > -2 * PI / 3) {
                current_direction = RoadDirection::RIGHT;
            }
            else if(delta_heading <= PI / 3 && delta_heading >= -PI / 3) {
                current_direction = RoadDirection::STRAIGHT;
            }
            else {
                current_direction = RoadDirection::UTURN;
            }
            for(int j = entry_index; j <= exit_index; ++j) {
                global_path[i].direction = current_direction;
            }
        }
    }
}

int MapManager::getGlobalPathNearestIndex() const {
    UtmPosition  vehicle_position = map.nav_info.car_pose.utm_position;
    double       min_dis          = 1e10;
    int          min_idx          = -1;
    const double angle_tolerance  = 1.7;
    const double dis_tolerance    = 400;
    double       dx, dy, da, dis;
    for(int i = 0; i < global_path.size(); ++i) {
        dx = vehicle_position.utm_x - global_path[i].utm_position.utm_x;
        dy = vehicle_position.utm_y - global_path[i].utm_position.utm_y;
        da = fabs(vehicle_position.heading - global_path[i].utm_position.heading);
        if(da < angle_tolerance) {
            dis = sqrt(dx * dx + dy * dy);
            if(dis < min_dis) {
                min_dis = dis;
                min_idx = i;
            }
        }
    }
    if(min_dis >= dis_tolerance) return -1;
    return min_idx;
}

void MapManager::updateRefPath(bool need_opposite) {
    map.ref_path.clear();
    map.forward_ref_path.clear();
    const int    search_depth               = 500;
    const int    search_history_depth       = 150;
    double       min_dis                    = 1e10;
    const double angle_tolerance            = 1.7;
    bool         global_nearest_idx_updated = false;

    if(global_path_nearest_idx < 0) {
        global_path_nearest_idx    = getGlobalPathNearestIndex();
        global_nearest_idx_updated = true;
        if(global_path_nearest_idx < 0) return;
    }

    int  search_begin            = max(global_path_nearest_idx - search_history_depth, 0);
    int  search_end              = min(int(global_path.size()), global_path_nearest_idx + search_depth);
    Pose car_pose                = map.nav_info.car_pose;
    int  current_idx_in_ref_path = 0;
    for(int i = search_begin; i < search_end; ++i) {
        HDMapPoint p = global_path[i];
        p.updateLocalCoordinate(car_pose);
        double dx = p.utm_position.utm_x - car_pose.utm_position.utm_x;
        double dy = p.utm_position.utm_y - car_pose.utm_position.utm_y;
        double da = fabs(car_pose.utm_position.heading - p.utm_position.heading);

        if(da < angle_tolerance) {
            double dis = dx * dx + dy * dy;
            if(dis < min_dis) {
                min_dis                 = dis;
                global_path_nearest_idx = i;
            }
        }

        if(!p.in_map()) {
            if(!need_opposite && map.ref_path.size() > 5)
                break;
            else
                continue;
        }

        map.ref_path.emplace_back(p);
        if(i == global_path_nearest_idx) current_idx_in_ref_path = map.ref_path.size() - 1;
    }

    if(map.ref_path.empty()) {
        if(global_nearest_idx_updated) return;
        global_path_nearest_idx = -1;
        updateRefPath(need_opposite);
        return;
    }
    else {
        map.ref_path.front().s = 0;
        for(int i = 0; i < map.ref_path.size(); ++i) {
            double delta_s    = hypot(fabs(map.ref_path[i].x - map.ref_path[i - 1].x), fabs(map.ref_path[i].y - map.ref_path[i - 1].y)) * GRID_RESOLUTION;
            map.ref_path[i].s = map.ref_path[i - 1].s + delta_s;
        }

        double current_position_s = map.ref_path[current_idx_in_ref_path].s;
        for(auto& p : map.ref_path) {
            p.s -= current_position_s;
        }
    }
    adjustRefPathByVLaneLine();
    for(const auto& p : map.ref_path) {
        if(p.s >= 0) map.forward_ref_path.push_back(p);
    }
    getLaneLineList();
    laneMatch();
    getBoundaryLine();
}

void MapManager::avoidPedestrian() {
    for(const auto& obj : map.dynamic_obj_list.dynamic_obj_list) {
        if(obj.type != ObjectType::PEDESTRIAN) continue;
        Pose   obj_pos               = obj.path.front();
        double dis_to_right_boundary = point2LineDis(obj_pos, map.boundary_line[0]);
        if(dis_to_right_boundary < 0) continue;
        double dis_to_left_boundary = point2LineDis<Pose, LinePoint>(obj_pos, map.boundary_line[1]);
        if(dis_to_right_boundary > 0) continue;
        double dis_to_maintained_path = point2LineDis<Pose, Pose>(obj_pos, map.maintained_path);
        if(fabs(dis_to_maintained_path) < map.forward_ref_path.front().lane_width) {
            int        shortest_point_index = shortestPointIndex<Pose, Pose>(obj_pos, map.maintained_path);
            int        x                    = map.maintained_path[shortest_point_index].x;
            int        y                    = map.maintained_path[shortest_point_index].y;
            DynamicObj dummy_obj;
            dummy_obj.width  = 1.5;
            dummy_obj.length = 3;
            dummy_obj.path.emplace_back(Pose(x, y, PI, 0, 0, 0));
            map.dynamic_obj_list.dynamic_obj_list.emplace_back(dummy_obj);
        }
    }
}

void MapManager::updatePlanningMap(LaneLineBlockType lane_line_block_type) {
    memset(map.line_block_map, 0, sizeof(map.line_block_map));
    if(lane_line_block_type == LaneLineBlockType::NO_BLOCK)
        ;
    // block uncrrect lane in intersection
    else {
        for(const auto& p : map.ref_path) {
            if(p.mode == HDMapMode::INTERSECTION_SOLID) {
                if(p.lane_num < 2)
                    continue;
                else if(p.lane_num == 2) {
                    double base_dis = 0;
                    if(p.direction == RoadDirection::RIGHT || p.direction == RoadDirection::STRAIGHT)
                        base_dis = p.lane_num - p.lane_seq - 0.5 * p.lane_width;
                    else
                        base_dis = p.lane_num - p.lane_seq - 1.5 * p.lane_width;
                    int k        = p.lane_width / 0.5;
                    for(int i = 1; i < k; ++i) {
                        Pose block_p                                       = p.getLateralPose(base_dis + i * 0.5);
                        map.line_block_map[int(block_p.x)][int(block_p.y)] = 1;
                    }
                }
                else {
                    double base_dis     = 0;
                    int    start_lane_i = 0;
                    int    end_lane_i   = 0;
                    if(p.direction == RoadDirection::RIGHT) {
                        start_lane_i = 2;
                        end_lane_i   = p.lane_num;
                    }
                    if(p.direction == RoadDirection::STRAIGHT) {
                        start_lane_i = 2;
                        end_lane_i   = p.lane_num - 1;
                    }
                    else {
                        start_lane_i = 1;
                        end_lane_i   = p.lane_num - 1;
                    }
                    for(int lane_i = start_lane_i; lane_i <= end_lane_i; ++lane_i) {
                        base_dis = lane_i - p.lane_seq - 0.5 * p.lane_width;
                        int k    = p.lane_width / 0.5;
                        for(int i = 1; i < k; ++i) {
                            Pose block_p                                       = p.getLateralPose(base_dis + i * 0.5);
                            map.line_block_map[int(block_p.x)][int(block_p.y)] = 1;
                        }
                    }
                }
            }
        }
        // block dash line and boundary
        if(lane_line_block_type == LaneLineBlockType::ALL_BLOCK) {
            for(const auto& line : map.lane_line_list) {
                for(const auto& p : line) {
                    if(p.type == LineType::SOLID) map.line_block_map[int(p.x)][int(p.y)] = 1;
                }
            }
            for(const auto& line : map.boundary_line) {
                for(const auto& p : line) {
                    if(p.type == LineType::BOUNDARY) map.line_block_map[int(p.x)][int(p.y)] = 1;
                }
            }
        }
        if(lane_line_block_type == LaneLineBlockType::SEMI_BLOCK) {
            for(const auto& line : map.lane_line_list) {
                for(int i = int(line.size() * 2 / 3); i < line.size(); ++i) {
                    auto& p                                                              = line[i];
                    if(p.type == LineType::SOLID) map.line_block_map[int(p.x)][int(p.y)] = 1;
                }
            }
            for(const auto& line : map.boundary_line) {
                for(int i = int(line.size() * 2 / 3); i < line.size(); ++i) {
                    auto& p                                                                 = line[i];
                    if(p.type == LineType::BOUNDARY) map.line_block_map[int(p.x)][int(p.y)] = 1;
                }
            }
        }
    }
    getPlanningDisMap();
    getAccessibleMap();
}

Map& MapManager::getMap() {
    return map;
}

void MapManager::handleLidarMap() {
    memset(map.lidar_map, 0, sizeof(map.lidar_map));
    memset(map.static_lidar_map, 0, sizeof(map.static_lidar_map));
    if(!map.lidar.detected) return;
    for(int r = 0; r < MAX_ROW; ++r) {
        for(int c = 0; c < MAX_COL; ++c) {
            if(map.rain_signal.signal) {
                map.lidar_map[r][c]        = (unsigned char)map.lidar.map[r][c] & 01;
                map.static_lidar_map[r][c] = (unsigned char)map.lidar.map[r][c] & 01;
                continue;
            }
            map.static_lidar_map[r][c] = map.lidar.map[r][c];
            if(map.lidar.map[r][c] & 04) continue;
            map.lidar_map[r][c] = map.lidar.map[r][c];
        }
    }
    if(map.rain_signal.signal) {
        // add obstacle in ESR
        // TODO
    }
}

void MapManager::getLidarDisMap() {
    const int    dx[]  = { 0, 0, -1, 1, 1, 1, -1, -1 };
    const int    dy[]  = { -1, 1, 0, 0, 1, -1, 1, -1 };
    const double dis[] = { 1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414 };

    for(int r = 0; r < MAX_ROW; ++r) {
        for(int c = 0; c < MAX_COL; ++c) {
            map.lidar_dis_map[r][c] = 1000000.0;
        }
    }
    queue<pair<int, int>> obj_que;

    for(int r = 0; r < MAX_ROW; ++r) {
        for(int c = 0; c < MAX_COL; ++c) {
            if(map.lidar_map[r][c] != 0 && map.lidar_map[r][c] != 0x4) {
                map.lidar_dis_map[r][c] = 0;
                obj_que.push(make_pair(r, c));
            }
        }
    }

    while(!obj_que.empty()) {
        const int x = obj_que.front().first;
        const int y = obj_que.front().second;
        obj_que.pop();
        for(int i = 0; i < 8; ++i) {
            const int tx = x + dx[i], ty = y + dy[i];
            if(Point2d(tx, ty).in_map() && map.lidar_dis_map[tx][ty] > map.lidar_dis_map[x][y] + dis[i]) {
                map.lidar_dis_map[tx][ty] = map.lidar_dis_map[x][y] + dis[i];
                obj_que.push(make_pair(tx, ty));
            }
        }
    }
}

void MapManager::getPlanningDisMap() {
    const int    dx[]  = { 0, 0, -1, 1, 1, 1, -1, -1 };
    const int    dy[]  = { -1, 1, 0, 0, 1, -1, 1, -1 };
    const double dis[] = { 1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414 };

    for(int r = 0; r < MAX_ROW; ++r) {
        for(int c = 0; c < MAX_COL; ++c) {
            map.planning_dis_map[r][c] = 1000000.0;
        }
    }
    queue<pair<int, int>> obj_que;

    for(int r = 0; r < MAX_ROW; ++r) {
        for(int c = 0; c < MAX_COL; ++c) {
            if(map.line_block_map[r][c] != 0 || (map.lidar_map[r][c] != 0 && map.lidar_map[r][c] != 0x4)) {
                map.planning_dis_map[r][c] = 0;
                obj_que.push(make_pair(r, c));
            }
        }
    }

    while(!obj_que.empty()) {
        const int x = obj_que.front().first;
        const int y = obj_que.front().second;
        obj_que.pop();
        for(int i = 0; i < 8; ++i) {
            const int tx = x + dx[i], ty = y + dy[i];
            if(Point2d(tx, ty).in_map() && map.planning_dis_map[tx][ty] > map.planning_dis_map[x][y] + dis[i]) {
                map.planning_dis_map[tx][ty] = map.planning_dis_map[x][y] + dis[i];
                obj_que.push(make_pair(tx, ty));
            }
        }
    }
}

void MapManager::getAccessibleMap() {
    const int dx[] = { 0, 0, -1, 1 };
    const int dy[] = { -1, 1, 0, 0 };

    memset(map.accessible_map, false, sizeof(map.accessible_map));
    queue<pair<int, int>> obj_que;
    map.accessible_map[CAR_CEN_ROW][CAR_CEN_COL] = true;
    obj_que.push(make_pair(CAR_CEN_ROW, CAR_CEN_COL));

    bool visited[MAX_ROW][MAX_COL];
    memset(visited, false, sizeof(visited));
    while(!obj_que.empty()) {
        const int x = obj_que.front().first;
        const int y = obj_que.front().second;
        obj_que.pop();
        for(int i = 0; i < 4; ++i) {
            const int tx = x + dx[i], ty = y + dy[i];
            if(!visited[tx][ty] && Point2d(tx, ty).in_map() && map.planning_dis_map[tx][ty] * GRID_RESOLUTION > COLLISION_CIRCLE_SMALL_R) {
                visited[tx][ty]            = true;
                map.accessible_map[tx][ty] = true;
                obj_que.push(make_pair(tx, ty));
            }
        }
    }
}

void MapManager::adjustRefPathByVLaneLine() {
    //获取视觉感知车道线list,从右往左排列
    map.v_line_list.clear();
    if(map.lane_list.detected) {
        map.v_line_list.push_back(map.lane_list.lane_list.front().right_line.points);
        for(const auto& lane : map.lane_list.lane_list) {
            map.v_line_list.push_back(lane.left_line.points);
        }
    }
    //通过边界车道线矫正参考路
    double     ref_offset_dis = 0;
    HDMapPoint first_p;
    for(auto& hd_p : map.ref_path) {
        if(hd_p.s >= 0) {
            first_p = hd_p;
            break;
        }
    }
    if(!map.v_line_list.empty()) {
        //优先以左边界线矫正
        if(map.v_line_list.back().front().type == LineType::BOUNDARY) {
            //计算参考路所指最左侧车道线的点
            double most_left_dis = first_p.lane_width * (first_p.lane_num - first_p.lane_seq + 0.5);
            Pose   most_left_p   = first_p.getLateralPose(most_left_dis);
            //计算最左侧车道线点和视觉检测边界线的偏移距离
            LinePoint v_1_p = map.v_line_list.back()[0];
            LinePoint v_2_p = map.v_line_list.back()[1];
            Point2d   vec_a = most_left_p - v_1_p;
            Point2d   vec_b = v_2_p - v_1_p;
            ref_offset_dis  = vec_a.cross(vec_b) / vec_b.len();
        }
        //其次以右边界矫正
        else if(map.v_line_list.front().front().type == LineType::BOUNDARY) {
            //计算参考路所指最右侧车道线的
            double most_right_dis = first_p.lane_width * (first_p.lane_seq - 1.5);
            Pose   most_right_p   = first_p.getLateralPose(most_right_dis);
            //计算最右侧车道线点和视觉检测边界线的偏移距离
            LinePoint v_1_p = map.v_line_list.front()[0];
            LinePoint v_2_p = map.v_line_list.front()[1];
            Point2d   vec_a = most_right_p - v_1_p;
            Point2d   vec_b = v_2_p - v_1_p;
            ref_offset_dis  = vec_a.cross(vec_b) / vec_b.len();
        }
        //要是左右都没有边界线，那就最近匹配
        else {
            //如果视觉车道线和地图车道线数量相同，直接左边界匹配
            if(map.v_line_list.size() == first_p.lane_num + 1) {
                double most_left_dis = first_p.lane_width * (first_p.lane_num - first_p.lane_seq + 0.5);
                Pose   most_left_p   = first_p.getLateralPose(most_left_dis);
                //计算最左侧车道线点和视觉检测边界线的偏移距离
                LinePoint v_1_p = map.v_line_list.back()[0];
                LinePoint v_2_p = map.v_line_list.back()[1];
                Point2d   vec_a = most_left_p - v_1_p;
                Point2d   vec_b = v_2_p - v_1_p;
                ref_offset_dis  = vec_a.cross(vec_b) / vec_b.len();
            }
            //如果视觉车道线少于地图车道线,直接右边界匹配效果好
            else if(map.v_line_list.size() < first_p.lane_num + 1) {
                double most_right_dis = first_p.lane_width * (first_p.lane_seq - 1.5);
                Pose   most_right_p   = first_p.getLateralPose(most_right_dis);
                //计算最右侧车道线点和视觉检测边界线的偏移距离
                LinePoint v_1_p = map.v_line_list.front()[0];
                LinePoint v_2_p = map.v_line_list.front()[1];
                Point2d   vec_a = most_right_p - v_1_p;
                Point2d   vec_b = v_2_p - v_1_p;
                ref_offset_dis  = vec_a.cross(vec_b) / vec_b.len();
            }
            //如果视觉车道线多于地图车道线,这种情况需要保证车辆在匹配后的车道内
            else {
                //先做最小距离和匹配，判断车辆是否在地图车道内
                //这种情况会有吗！实现好麻烦，先不管了TODO
            }
        }
        //矫正参考路
        if(ref_offset_dis != 0) {
            for(auto& p : map.ref_path) {
                //垂直偏移矫正-----------
                Point2d offset_vec = offsetPoint(p.getDirectionVec(), ref_offset_dis);
                p.x += offset_vec.x;
                p.y += offset_vec.y;
                //暂时不考虑曲率变化
                //只横向矫正--------------
                // p.y -= ref_offset_dis;
                //------------------------
            }
        }
    }
}

int MapManager::getCarLaneId() {
    HDMapPoint p_ref = map.ref_path.front();
    Point2d    p_car = Point2d(CAR_CEN_ROW, CAR_CEN_COL);
    Point2d    vec_a = p_car - p_ref.toPoint2d();
    Point2d    vec_b = p_ref.getDirectionVec();
    double     dis   = vec_b.cross(vec_a);
    return ceil((dis * GRID_RESOLUTION - p_ref.lane_width / 2) / p_ref.lane_width) + p_ref.lane_seq;
}

void MapManager::getLaneLineList() {
    //清空之前的lane信息
    map.lane_line_list.clear();
    map.lane_center_list.clear();
    //判断车道数是否不变
    bool lane_num_change = false;
    int  lane_num        = map.forward_ref_path.front().lane_num;
    for(const auto& p : map.forward_ref_path) {
        if(p.lane_num != lane_num) {
            lane_num_change = true;
            break;
        }
    }
    map.lane_line_list.resize(lane_num + 1);
    map.lane_center_list.resize(lane_num);
    for(const auto& p : map.forward_ref_path) {
        //从右向左第i条地图车道线
        if(p.lane_num != lane_num) break;
        for(int i = 0; i < lane_num + 1; ++i) {
            Pose     line_p                                                                                                            = p.getLateralPose(p.lane_width * (i - p.lane_seq + 0.5));
            LineType line_type                                                                                                         = LineType::DASH;
            if(p.mode == HDMapMode::INTERSECTION_SOLID) line_type                                                                      = LineType::SOLID;
            if(i == 0 && (p.block_type & BlockType::BlockRight) || (i == lane_num && (p.block_type & BlockType::BlockLeft))) line_type = LineType::BOUNDARY;
            if(line_p.in_map() && p.mode != HDMapMode::CHANGE) map.lane_line_list[i].emplace_back(line_p.x, line_p.y, line_type);
        }
        if(!lane_num_change) {
            //从右向左第i条地图车道中心线
            for(int i = 0; i < lane_num; ++i) {
                Pose line_p = p.getLateralPose(p.lane_width * (i - p.lane_seq + 1));
                if(line_p.in_map() && p.mode != HDMapMode::CHANGE) map.lane_center_list[i].emplace_back(line_p.x, line_p.y);
            }
        }
    }
    //若车道数改变了,获取远处的中心线,供选择目标点
    if(lane_num_change) {
        lane_num = map.forward_ref_path.back().lane_num;
        map.lane_center_list.resize(lane_num);
        for(int i = map.forward_ref_path.size() - 1; i >= 0; --i) {
            const HDMapPoint& p = map.forward_ref_path[i];
            if(p.lane_num != lane_num) break;
            //从右向左第j条地图车道中心线
            for(int j = 0; j < lane_num; ++j) {
                Pose line_p = p.getLateralPose(p.lane_width * (j - p.lane_seq + 1));
                if(line_p.in_map() && p.mode != HDMapMode::CHANGE) map.lane_center_list[j].emplace_back(line_p.x, line_p.y);
            }
        }
        for(auto& line : map.lane_center_list)
            reverse(line.begin(), line.end());
    }
}

vector<Pose> MapManager::getLaneTargets() {
    vector<Pose> targets;
    for(const auto& line : map.lane_center_list) {
        for(int i = line.size() - 1; i >= 0; --i) {
            const Point2d& p = line[i];
            if(map.accessible_map[int(p.x)][int(p.y)]) {
                double ang = PI;
                if(i - 1 >= 0) {
                    const Point2d& pp  = line[i - 1];
                    Point2d        vec = p - pp;
                    ang                = vec.getRad();
                }
                else if(i + 1 < line.size()) {
                    const Point2d& np  = line[i + 1];
                    Point2d        vec = np - p;
                    ang                = vec.getRad();
                }
                targets.push_back(Pose(p.x, p.y, ang));
                break;
            }
        }
    }
    return targets;
}

void MapManager::laneMatch() {
    //视觉检测到车道线匹配
    //若视觉检测和地图车道线数量一样
    if(map.lane_line_list.size() == map.v_line_list.size()) {
        for(int i = 0; i < map.lane_line_list.size(); ++i) {
            int       shortest_index = shortestPointIndex(map.nav_info.car_pose, map.lane_line_list[i]);
            LinePoint lp             = map.lane_line_list[i][shortest_index];
            LinePoint vp1            = map.v_line_list[i].front();
            LinePoint vp2            = map.v_line_list[i][1];
            Point2d   vec_a          = vp2 - vp1;
            Point2d   vec_b          = lp - vp1;
            double    offset         = (vec_b.cross(vec_a) / vec_a.len()) * GRID_RESOLUTION;
            Point2d   off_p;
            for(int j = 0; j < map.lane_line_list[i].size() - 1; ++j) {
                Point2d vec_d = map.lane_line_list[i][j + 1] - map.lane_line_list[i][j];
                off_p         = offsetPoint(vec_d, offset);
                map.lane_line_list[i][j].x += off_p.x;
                map.lane_line_list[i][j].y += off_p.y;
            }
            map.lane_line_list[i][map.lane_line_list.size() - 1].x += off_p.x;
            map.lane_line_list[i][map.lane_line_list.size() - 1].y += off_p.y;
        }
    }
    else {
        //如果车道线数量不一样，左边界对齐或者右边界对齐
        // TODO
    }
    laneLineInterpolation();
}

void MapManager::laneLineInterpolation() {
    for(auto& line : map.lane_line_list) {
        lineInterpolation<LinePoint>(line);
    }
    for(auto& line : map.lane_center_list) {
        lineInterpolation<Point2d>(line);
    }
}

void MapManager::getBoundaryLine() {
    map.boundary_line.clear();
    bool lane_num_change     = false;
    int  lane_line_change_id = -1;
    for(int i = 0; i < map.forward_ref_path.size(); ++i) {
        if(map.forward_ref_path[i].lane_num != map.forward_ref_path.front().lane_num) {
            lane_num_change     = true;
            lane_line_change_id = i;
            break;
        }
    }
    vector<LinePoint> right_boundary;
    vector<LinePoint> left_boundary;
    for(auto p : map.ref_path) {
        if(p.s >= 0 || p.mode == HDMapMode::CHANGE) break;
        Pose     right_line_p                                    = p.getLateralPose(p.lane_width * (-p.lane_seq + 0.5));
        LineType right_line_type                                 = LineType::DASH;
        if(p.block_type & BlockType::BlockRight) right_line_type = LineType::BOUNDARY;
        if(right_line_p.in_map()) right_boundary.emplace_back(right_line_p.x, right_line_p.y, right_line_type);
        Pose     left_line_p                                   = p.getLateralPose(p.lane_width * (p.lane_num - p.lane_seq + 0.5));
        LineType left_line_type                                = LineType::DASH;
        if(p.block_type & BlockType::BlockLeft) left_line_type = LineType::BOUNDARY;
        if(left_line_p.in_map()) left_boundary.emplace_back(left_line_p.x, left_line_p.y, left_line_type);
    }
    for(auto p : map.lane_line_list.front())
        right_boundary.push_back(p);
    for(auto p : map.lane_line_list.back())
        left_boundary.push_back(p);
    if(lane_num_change) {
        for(int j = lane_line_change_id; j < map.forward_ref_path.size(); ++j) {
            HDMapPoint p = map.forward_ref_path[j];
            if(p.mode == HDMapMode::CHANGE) continue;
            Pose right_line_p = p.getLateralPose(p.lane_width * (-p.lane_seq + 0.5));
            if(right_line_p.in_map()) right_boundary.emplace_back(right_line_p.x, right_line_p.y, LineType::BOUNDARY);
            Pose left_line_p = p.getLateralPose(p.lane_width * (p.lane_num - p.lane_seq + 0.5));
            if(left_line_p.in_map()) left_boundary.emplace_back(left_line_p.x, left_line_p.y, LineType::BOUNDARY);
        }
    }
    map.boundary_line.push_back(right_boundary);
    map.boundary_line.push_back(left_boundary);
    for(auto& line : map.boundary_line) {
        lineInterpolation(line);
        for(int n = 1; n < line.size(); ++n) {
            if(line[n].type == LineType::UNKNOWN_LINE) line[n].type = line[n - 1].type;
        }
    }
}

void MapManager::visualization() {
    MessageManager*   msgm = MessageManager::getInstance();
    visVISUALIZATION& vis  = msgm->visualization;
    // reference path
    vis.reference_path.clear();
    vis.reference_path_size = map.forward_ref_path.size();
    for(const auto& p : map.forward_ref_path) {
        visPoint vp;
        vp.x = p.x;
        vp.y = p.y;
        vis.reference_path.push_back(vp);
    }
    // planning map
    memcpy(vis.safe_map, map.planning_dis_map, sizeof(map.planning_dis_map));
    // best path
    vis.best_path.clear();
    vis.best_path_size = map.best_path.path.size();
    for(const auto& p : map.best_path.path) {
        visPoint vp;
        vp.x = p.x;
        vp.y = p.y;
        vis.best_path.push_back(vp);
    }
    // maintained path
    vis.maintained_path.clear();
    vis.maintained_path_size = map.maintained_path.size();
    for(const auto& p : map.maintained_path) {
        visPoint vp;
        vp.x = p.x;
        vp.y = p.y;
        vis.maintained_path.push_back(vp);
    }
    // lanes and boundary
    vis.lanes.clear();
    vis.lanes_size = map.lane_line_list.size() + map.boundary_line.size();
    for(const auto& lane_line : map.lane_line_list) {
        visLaneLine vll;
        vll.lane_line_points_size = lane_line.size();
        for(const auto& p : lane_line) {
            visPoint vp;
            vp.x = p.x;
            vp.y = p.y;
            vll.lane_line_points.push_back(vp);
        }
        vis.lanes.push_back(vll);
    }
    for(const auto& boundary_line : map.boundary_line) {
        visLaneLine vll;
        vll.lane_line_points_size = boundary_line.size();
        for(const auto& p : boundary_line) {
            visPoint vp;
            vp.x = p.x;
            vp.y = p.y;
            vll.lane_line_points.push_back(vp);
        }
        vis.lanes.push_back(vll);
    }
    // speed planner
    msgm->setSpeedPath(map.best_path);
    msgm->publishVisualization();
}

vector<Pose> MapManager::getStartMaintainedPath() {
    maintained_path_mutex.lock_shared();
    vector<Pose> path = maintained_path;
    maintained_path_mutex.unlock_shared();
    double maintained_s = 2 * map.nav_info.current_speed;
    for(auto p : path)
        p.updateLocalCoordinate(map.nav_info.car_pose);
    int          shortest_index = shortestPointIndex(map.nav_info.car_pose, path);
    vector<Pose> res;
    for(auto p : path) {
        p.s -= path[shortest_index].s;
        if(p.s >= 0 && p.s < maintained_s) res.push_back(p);
    }
    return res;
}

vector<Pose> MapManager::getMaintainedPath() {
    maintained_path_mutex.lock_shared();
    vector<Pose> path = maintained_path;
    maintained_path_mutex.unlock_shared();
    for(auto p : path)
        p.updateLocalCoordinate(map.nav_info.car_pose);
    int          shortest_index = shortestPointIndex(map.nav_info.car_pose, path);
    vector<Pose> res;
    for(auto p : path) {
        p.s -= path[shortest_index].s;
        if(p.s >= 0) res.push_back(p);
    }
    return res;
}

void MapManager::maintainPath() {
    maintained_path_mutex.lock();
    maintained_path.clear();
    maintained_path.reserve(map.best_path.path.size());
    Pose car_pose = map.nav_info.car_pose;
    for(auto p : map.best_path.path) {
        p.updateGlobalCoordinate(car_pose);
        maintained_path.push_back(p);
    }
    maintained_path_mutex.unlock();
}

void MapManager::selectBestPath(const vector<SpeedPath>& paths) {
    map.best_path = SpeedPath();
    if(paths.empty()) return;
    int best_speed_path_index = -1;
    int max_index             = -1;
    for(int i = 0; i < paths.size(); ++i) {
        const SpeedPath& speed_path = paths[i];
        const auto&      path       = speed_path.path;
        auto comp                   = [](const Pose& p1, const double t) { return p1.t < t; };
        auto itr                    = lower_bound(path.begin(), path.end(), 5, comp);
        Pose p;
        if(itr != path.end())
            p = *itr;
        else
            p              = path.back();
        int ref_path_index = shortestPointIndex(p, map.ref_path);
        if(ref_path_index > max_index) {
            max_index             = ref_path_index;
            best_speed_path_index = i;
        }
    }
    if(best_speed_path_index >= 0) map.best_path = paths[best_speed_path_index];
}
MapManager* MapManager::instance = new MapManager;
}  // namespace TiEV