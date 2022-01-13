#include "map_manager.h"

#include <math.h>

#include <fstream>
#include <iostream>
#include <queue>
#include <shared_mutex>

#include "collision_check.h"
#include "lattice_planner.h"
#include "tiev_utils.h"
#include "tievlog.h"

namespace TiEV {
using namespace std;

void MapManager::update() {
  MessageManager& message_manager = MessageManager::getInstance();
  message_manager.getNavInfo(map.nav_info);
  message_manager.getSlamInfo(map.slam_info);
  message_manager.getTrafficLight(map.traffic_light);
  message_manager.getDynamicObjList(map.dynamic_obj_list);
  message_manager.getParkingLotList(map.parking_lot_list);
  message_manager.getLaneList(map.lane_list);
  message_manager.getRainSignal(map.rain_signal);
  message_manager.getMap(map.lidar);
  //-------
  updateRefPath();
  handleLidarMap();
}

double MapManager::getSpeedBySpeedMode(int speed_mode) {
  const auto& cfg = Config::getInstance();
  if (cfg.control_mode == ControlMode::PlanningWithDebugMode ||
      cfg.control_mode == ControlMode::TrakingWithDebugMode)
    speed_mode = cfg.debug_speed_mode;
  switch (speed_mode) {
    case 0:
      return cfg.back_speed / 3.6;
    case 1:
      return cfg.stop_speed / 3.6;
    case 2:
      return cfg.very_low_speed / 3.6;
    case 3:
      return cfg.low_speed / 3.6;
    case 4:
      return cfg.mid_speed / 3.6;
    case 5:
      return cfg.high_speed / 3.6;
    case 6:
      return cfg.very_high_speed / 3.6;
    default:
      return cfg.stop_speed / 3.6;
  }
}

double MapManager::getCurrentMapSpeed() {
  ref_path_mutex.lock_shared();
  if (map.forward_ref_path.empty()) {
    ref_path_mutex.unlock_shared();
    return 0;
  }
  double res = getSpeedBySpeedMode(map.forward_ref_path.front().speed_mode);
  ref_path_mutex.unlock_shared();
  return res;
}

bool MapManager::requestGlobalPath(const NavInfo& nav_info) {
  cout << "request global path not implimented" << endl;
  return false;
}

void MapManager::readGlobalPathFile(const string& file_path) {
  const string error_header = "ERROR:Read road-map file failed : ";

  std::unique_lock<std::shared_mutex> locker(global_path_mutex);
  global_path.clear();
  fstream input(file_path, ios::in);
  if (!input.is_open()) {
    cout << error_header << file_path << " not exist." << endl;
    return;
  }

  string buffer, field;
  getline(input, buffer);
  stringstream buffer_stream(buffer);

  // check number of fields
  const int field_num         = 14;
  int       field_num_in_file = 0;
  while (buffer_stream >> field) {
    cout << "Add field: " << field << endl;
    ++field_num_in_file;
  }

  if (field_num_in_file != field_num) {
    cout << error_header << "content of input file is in illegal format."
         << endl;
    return;
  }

  double lon, lat, utm_x, utm_y, heading, curve, lane_width;
  int    id, mode, speed_mode, event, block_type, lane_num, lane_seq;
  while (input >> id >> lon >> lat >> utm_x >> utm_y >> heading >> curve >>
         mode >> speed_mode >> event >> block_type >> lane_num >> lane_seq >>
         lane_width) {
    HDMapPoint point(lon, lat, utm_x, utm_y, heading, curve, (HDMapMode)mode,
                     (HDMapSpeed)speed_mode, (HDMapEvent)event,
                     (BlockType)block_type, lane_num, lane_seq, lane_width);
    global_path.push_back(point);
  }
  filtPoints();
  setGlobalPathDirection();
  LOG(INFO) << "Map file already loaded!Enjoy!";
}

vector<Task> MapManager::getCurrentTasks() {
  task_mutex.lock_shared();
  vector<Task> res = this->map.current_tasks;
  task_mutex.unlock_shared();
  return res;
}

Task MapManager::getParkingTask() {
  parking_task_mutex.lock_shared();
  Task res = this->map.parking_task;
  parking_task_mutex.unlock_shared();
  return res;
}
void MapManager::popCurrentTask() {
  if (this->map.current_tasks.empty()) return;
  task_mutex.lock();
  this->map.current_tasks.pop_back();
  task_mutex.unlock();
}

void MapManager::pushCurrentTask(const Task& next_task) {
  task_mutex.lock_shared();
  this->map.current_tasks.push_back(next_task);
  task_mutex.unlock_shared();
}

void MapManager::clearTask() {
  task_mutex.lock();
  this->map.current_tasks.clear();
  task_mutex.unlock();
}

void MapManager::setGlobalPath(const vector<HDMapPoint>& new_global_path) {
  global_path_mutex.lock();
  this->global_path = new_global_path;
  filtPoints();
  setGlobalPathDirection();
  global_path_nearest_idx = -1;
  global_path_mutex.unlock();
}

void MapManager::filtPoints() {
  vector<HDMapPoint> tmp_points;
  tmp_points.reserve(global_path.size());
  double priv_ang = 0, cur_ang = 0, next_ang = 0;
  int    priv_lane = 0, cur_lane = 0, next_lane = 0;
  if (!global_path.empty()) tmp_points.push_back(global_path.front());

  for (int i = 1; i + 1 < global_path.size(); ++i) {
    if (global_path[i].event != HDMapEvent::ENTRY_INTERSECTION &&
        global_path[i].event != HDMapEvent::EXIT_INTERSECTION) {
      priv_ang         = global_path[i - 1].utm_position.heading;
      cur_ang          = global_path[i].utm_position.heading;
      next_ang         = global_path[i + 1].utm_position.heading;
      double delta_ang = priv_ang - next_ang;
      normalizeAngle(delta_ang);
      double std_ang = delta_ang / 2.0 + next_ang;
      normalizeAngle(std_ang);
      normalizeAngle(cur_ang);
      if (fabs(cur_ang - std_ang) > 0.6) continue;

      priv_lane = global_path[i - 1].lane_num;
      cur_lane  = global_path[i].lane_num;
      next_lane = global_path[i + 1].lane_num;
      if (priv_lane == next_lane && next_lane != cur_lane) continue;
    }

    tmp_points.push_back(global_path[i]);
  }

  if (global_path.size() > 1) tmp_points.push_back(global_path.back());

  swap(global_path, tmp_points);
}

void MapManager::setGlobalPathDirection() {
  RoadDirection current_direction = RoadDirection::STRAIGHT;
  double        heading_after = 0.0, heading_before = 0.0;
  int           exit_index, entry_index;
  for (int i = global_path.size(); i >= 0; i--) {
    if (global_path[i].event != HDMapEvent::ENTRY_INTERSECTION &&
        global_path[i].event != HDMapEvent::EXIT_INTERSECTION) {
      global_path[i].direction = current_direction;
    } else if (global_path[i].event == HDMapEvent::EXIT_INTERSECTION) {
      heading_after = global_path[i].utm_position.heading;
      exit_index    = i;
    } else if (global_path[i].event == HDMapEvent::ENTRY_INTERSECTION) {
      heading_before       = global_path[i].utm_position.heading;
      entry_index          = i;
      double delta_heading = heading_after - heading_before;
      while (delta_heading < -PI) delta_heading += 2 * PI;
      while (delta_heading >= PI) delta_heading -= 2 * PI;
      if (delta_heading > PI / 3 && delta_heading < 2 * PI / 3) {
        current_direction = RoadDirection::LEFT;
      } else if (delta_heading < -PI / 3 && delta_heading > -2 * PI / 3) {
        current_direction = RoadDirection::RIGHT;
      } else if (delta_heading <= PI / 3 && delta_heading >= -PI / 3) {
        current_direction = RoadDirection::STRAIGHT;
      } else {
        current_direction = RoadDirection::UTURN;
      }
      for (int j = entry_index; j <= exit_index; ++j) {
        global_path[i].direction = current_direction;
      }
    }
  }
}

vector<Pose> MapManager::getUTurnTargets() {
#if 0
    if(!maintained_uturn_target.empty()) {
        for(auto& target : maintained_uturn_target) {
            target.updateLocalCoordinate(map.nav_info.car_pose);
        }
        return maintained_uturn_target;
    }
    vector<HDMapPoint> tmp;
    for(int i = 0; i < map.ref_path.size(); ++i) {
        HDMapPoint point     = map.ref_path[i];
        double     delta_cos = point.cosDeltaAngle(map.nav_info.car_pose);
        if(delta_cos < cos(PI * 2 / 3)) tmp.push_back(point);
    }
    int shortest_index_on_rev_path = shortestPointIndex(map.nav_info.car_pose, tmp);
    int shortest_index_on_ref_path = shortestPointIndex(map.nav_info.car_pose, map.ref_path);
    if(shortest_index_on_rev_path >= 0) {
        double x = tmp[shortest_index_on_rev_path].x;
        double y = tmp[shortest_index_on_rev_path].y;
        double a = tmp[shortest_index_on_rev_path].ang;
        maintained_uturn_target.push_back(Pose(x, y, a));
    }
    else if(shortest_index_on_ref_path >= 0) {
        double x = map.ref_path[shortest_index_on_ref_path].x;
        double y = map.ref_path[shortest_index_on_ref_path].y;
        double a = map.ref_path[shortest_index_on_ref_path].ang;
        maintained_uturn_target.push_back(Pose(x, y, a));
    }
    for(auto&p: maintained_uturn_target) p.updateGlobalCoordinate(map.nav_info.car_pose);

    return maintained_uturn_target;
#endif
  if (!maintained_uturn_target.empty()) {
    for (auto& target : maintained_uturn_target) {
      target.updateLocalCoordinate(map.nav_info.car_pose);
    }
    return maintained_uturn_target;
  }
  if (map.forward_ref_path.empty()) return maintained_uturn_target;
  for (int i = map.forward_ref_path.size() - 1; i >= 0; --i) {
    HDMapPoint point = map.forward_ref_path[i];
    if (map.accessible_map[int(point.x)][int(point.y)]) {
      maintained_uturn_target.push_back(Pose(point.x, point.y, point.ang));
      break;
    }
  }
  for (auto& p : maintained_uturn_target)
    p.updateGlobalCoordinate(map.nav_info.car_pose);
  return maintained_uturn_target;
}

const int MapManager::getGlobalPathNearestIndex(const int begin,
                                                const int end) const {
  UtmPosition vehicle_position = map.nav_info.car_pose.utm_position;
  double      min_dis          = std::numeric_limits<double>::max();
  int         min_idx          = -1;
  double      dx, dy, da, dis;
  for (int i = std::max(0, begin);
       i < std::min(int(global_path.size()), end + 1); ++i) {
    dx  = vehicle_position.utm_x - global_path[i].utm_position.utm_x;
    dy  = vehicle_position.utm_y - global_path[i].utm_position.utm_y;
    dis = dx * dx + dy * dy;
    if (dis < min_dis) {
      min_dis = dis;
      min_idx = i;
    }
  }
  if (min_idx >= 0) {
    HDMapPoint global_p = global_path[min_idx];
    global_p.updateLocalCoordinate(map.nav_info.car_pose);
    if (!global_p.in_map()) return -1;
  }
  return min_idx;
}

HDMapMode MapManager::getCurrentMapMode() {
  ref_path_mutex.lock_shared();
  HDMapMode mode;
  if (map.forward_ref_path.empty())
    mode = HDMapMode::UNKNOWN_MODE;
  else
    mode = map.forward_ref_path.front().mode;
  ref_path_mutex.unlock_shared();
  return mode;
}

RoadDirection MapManager::getCurrentRoadDirection() {
  ref_path_mutex.lock_shared();
  RoadDirection road_direction;
  if (map.forward_ref_path.empty())
    road_direction = RoadDirection::STRAIGHT;
  else
    road_direction = map.forward_ref_path.front().direction;
  ref_path_mutex.unlock_shared();
  return road_direction;
}

HDMapPoint MapManager::getStopLine() {
  ref_path_mutex.lock_shared();
  HDMapPoint stop_line;
  for (const auto& p : map.forward_ref_path) {
    if (p.event != HDMapEvent::ENTRY_INTERSECTION) continue;
    stop_line = p;
    break;
  }
  ref_path_mutex.unlock_shared();
  return stop_line;
}

void MapManager::updateRefPath(bool need_opposite) {
  global_path_mutex.lock_shared();
  ref_path_mutex.lock();
  map.ref_path.clear();
  map.forward_ref_path.clear();
  const int search_depth         = 300;
  const int search_history_depth = 90;
  double    min_dis              = 1e10;

  if (global_path_nearest_idx < 0) {
    global_path_nearest_idx =
        getGlobalPathNearestIndex(0, global_path.size() - 1);
    if (global_path_nearest_idx < 0) {
      ref_path_mutex.unlock();
      global_path_mutex.unlock_shared();
      return;
    }
  }

  int search_begin = max(global_path_nearest_idx - search_history_depth, 0);
  int search_end =
      min(int(global_path.size() - 1), global_path_nearest_idx + search_depth);
  global_path_nearest_idx = getGlobalPathNearestIndex(search_begin, search_end);
  search_begin = max(global_path_nearest_idx - search_history_depth, 0);
  search_end =
      min(int(global_path.size() - 1), global_path_nearest_idx + search_depth);
  Pose car_pose                = map.nav_info.car_pose;
  int  current_idx_in_ref_path = 0;
  for (int i = search_begin; i <= search_end; ++i) {
    HDMapPoint p = global_path[i];
    p.updateLocalCoordinate(car_pose);
    p.v = getSpeedBySpeedMode(p.speed_mode);
    if (!p.in_map()) continue;
    map.ref_path.push_back(p);
    if (i == global_path_nearest_idx)
      current_idx_in_ref_path = int(map.ref_path.size()) - 1;
  }
  global_path_mutex.unlock_shared();

  if (map.ref_path.empty())
    global_path_nearest_idx = -1;
  else {
    map.ref_path.front().s = 0;
    for (int i = 1; i < map.ref_path.size(); ++i) {
      double delta_s = hypot(fabs(map.ref_path[i].x - map.ref_path[i - 1].x),
                             fabs(map.ref_path[i].y - map.ref_path[i - 1].y)) *
                       GRID_RESOLUTION;
      map.ref_path[i].s = map.ref_path[i - 1].s + delta_s;
    }

    double current_position_s = map.ref_path[current_idx_in_ref_path].s;
    for (auto& p : map.ref_path) {
      p.s -= current_position_s;
    }
  }
  adjustRefPathByVLaneLine();
  for (const auto& p : map.ref_path) {
    if (p.s >= 0) map.forward_ref_path.push_back(p);
  }
  ref_path_mutex.unlock();
  // laneMatch();
}

void MapManager::addPedestrian(DynamicObjList& dynamic) {
  global_path_mutex.lock_shared();
  const auto forward_ref_path = map.forward_ref_path;
  global_path_mutex.unlock_shared();
  for (const auto& p : forward_ref_path) {
    if (p.event != HDMapEvent::ENTRY_INTERSECTION &&
        p.event != HDMapEvent::EXIT_INTERSECTION)
      continue;
    int        x = p.x;
    int        y = p.y;
    DynamicObj dummy_obj;
    dummy_obj.width  = 10;
    dummy_obj.length = 1.5;
    dummy_obj.path.emplace_back(x, y, p.ang, 0, 0, 0);
    dynamic.dynamic_obj_list.emplace_back(dummy_obj);
    std::cout << "block stop line in decision.cpp " << std::endl;
    break;
  }
}

void MapManager::addPedestrian(DynamicObjList&           dynamic_obj_list,
                               const vector<HDMapPoint>& ref_path) {
  const int tmp_inf = 1e8;
  int       idx     = tmp_inf;
  if (ref_path.empty()) return;
  for (const auto& obj : dynamic_obj_list.dynamic_obj_list) {
    if (obj.type != ObjectType::PEDESTRIAN) continue;
    Pose obj_init_pose  = obj.path.front();
    Pose obj_final_pose = obj.path.back();

    double init_pose_dis  = point2LineDis(obj_init_pose, ref_path);
    double final_pose_dis = point2LineDis(obj_final_pose, ref_path);
    if (init_pose_dis * final_pose_dis >= 0 &&
        fabs(final_pose_dis) > fabs(init_pose_dis))
      continue;

    int init_shortest_point_index = shortestPointIndex(obj_init_pose, ref_path);
    HDMapPoint mapped_p           = ref_path[init_shortest_point_index];
    double     right_dis = (-mapped_p.lane_seq + 0.5) * mapped_p.lane_width;
    double     left_dis =
        (mapped_p.lane_num - mapped_p.lane_seq + 0.5) * mapped_p.lane_width;
    if (init_pose_dis <= right_dis ||
        init_pose_dis >= left_dis + mapped_p.lane_width)
      continue;

    idx = min(idx, init_shortest_point_index);
    int final_shortest_point_index =
        shortestPointIndex(obj_final_pose, ref_path);
    idx = min(idx, final_shortest_point_index);
  }
  if (idx == tmp_inf) return;

  DynamicObj tmp_obj;
  tmp_obj.heading = ref_path[idx].ang;
  tmp_obj.length  = 3;
  tmp_obj.width   = 20;
  tmp_obj.path.push_back(
      Pose(ref_path[idx].x, ref_path[idx].y, tmp_obj.heading));
  dynamic_obj_list.dynamic_obj_list.push_back(tmp_obj);
}

void MapManager::blockStopLine() {
  for (const auto& p : map.forward_ref_path) {
    if (p.event != HDMapEvent::ENTRY_INTERSECTION &&
        p.event != HDMapEvent::EXIT_INTERSECTION)
      continue;
    int        x = p.x;
    int        y = p.y;
    DynamicObj dummy_obj;
    dummy_obj.width  = 1.5;
    dummy_obj.length = 10;
    dummy_obj.path.emplace_back(x, y, p.ang, 0, 0, 0);
    map.dynamic_obj_list.dynamic_obj_list.emplace_back(dummy_obj);
    break;
  }
}

void MapManager::updatePlanningMap(DynamicBlockType dynamic_block_type,
                                   bool             history) {
  laneBlockDecision();
  dynamicDecision(dynamic_block_type);
  mapDecision(history);
  getAccessibleMap();
}

Map& MapManager::getMap() { return map; }

vector<HDMapPoint> MapManager::getForwardRefPath() {
  ref_path_mutex.lock_shared();
  vector<HDMapPoint> res = map.forward_ref_path;
  ref_path_mutex.unlock_shared();
  return res;
}

vector<HDMapPoint> MapManager::getRefPath() {
  ref_path_mutex.lock_shared();
  vector<HDMapPoint> res = map.ref_path;
  ref_path_mutex.unlock_shared();
  return res;
}

HDMapSpeed MapManager::getCurrentSpeedMode() {
  ref_path_mutex.lock_shared();
  vector<HDMapPoint> res = map.forward_ref_path;
  ref_path_mutex.unlock_shared();
  if (res.empty()) return HDMapSpeed::STOP_SPEED;
  return res.front().speed_mode;
}

void MapManager::handleLidarMap() {
  const int    dx[]  = {0, 0, -1, 1, 1, 1, -1, -1};
  const int    dy[]  = {-1, 1, 0, 0, 1, -1, 1, -1};
  const double dis[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      map.lidar_dis_map[r][c] = 1000000.0;
    }
  }
  queue<pair<int, int>> obj_que;
  memset(map.lidar_map, 0, sizeof(map.lidar_map));
  memset(map.static_lidar_map, 0, sizeof(map.static_lidar_map));
  if (!map.lidar.detected) return;
  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      if (map.rain_signal.detected && map.rain_signal.signal) {
        map.lidar_map[r][c]        = (unsigned char)map.lidar.map[r][c] & 01;
        map.static_lidar_map[r][c] = (unsigned char)map.lidar.map[r][c] & 01;
        if (map.lidar_map[r][c] != 0) {
          map.lidar_dis_map[r][c] = 0;
          obj_que.push(make_pair(r, c));
        }
        continue;
      }
      map.static_lidar_map[r][c] = map.lidar.map[r][c];
      if (map.lidar.map[r][c] & 0x04) continue;
      map.lidar_map[r][c] = map.lidar.map[r][c];
      if (map.lidar_map[r][c] != 0) {
        map.lidar_dis_map[r][c] = 0;
        obj_que.push(make_pair(r, c));
      }
    }
  }
  if (map.rain_signal.detected && map.rain_signal.signal) {
    // add obstacle in ESR
    // TODO
  }
  while (!obj_que.empty()) {
    const int x = obj_que.front().first;
    const int y = obj_que.front().second;
    obj_que.pop();
    for (int i = 0; i < 8; ++i) {
      const int tx = x + dx[i], ty = y + dy[i];
      if (Point2d(tx, ty).in_map() &&
          map.lidar_dis_map[tx][ty] > map.lidar_dis_map[x][y] + dis[i]) {
        map.lidar_dis_map[tx][ty] = map.lidar_dis_map[x][y] + dis[i];
        obj_que.push(make_pair(tx, ty));
      }
    }
  }
}

void MapManager::mapDecision(bool history) {
  const int    dx[]  = {0, 0, -1, 1, 1, 1, -1, -1};
  const int    dy[]  = {-1, 1, 0, 0, 1, -1, 1, -1};
  const double dis[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      map.planning_dis_map[r][c] = 1e6;
    }
  }
  queue<pair<int, int>> obj_que;

  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      if ((history && map.lidar_map[r][c] == 0x1) ||
          (map.lane_block_map[r][c] != 0 || map.dynamic_obs_map[r][c] != 0)) {
        // if we just use the history map
        map.planning_dis_map[r][c] = 0;
        obj_que.push(make_pair(r, c));
      } else if ((map.lidar_map[r][c] != 0x4 && map.lidar_map[r][c] != 0) ||
                 (map.lane_block_map[r][c] != 0 ||
                  map.dynamic_obs_map[r][c] != 0)) {
        map.planning_dis_map[r][c] = 0;
        obj_que.push(make_pair(r, c));
      }
    }
  }

  while (!obj_que.empty()) {
    const int x = obj_que.front().first;
    const int y = obj_que.front().second;
    obj_que.pop();
    for (int i = 0; i < 8; ++i) {
      const int tx = x + dx[i], ty = y + dy[i];
      if (Point2d(tx, ty).in_map() &&
          map.planning_dis_map[tx][ty] > map.planning_dis_map[x][y] + dis[i]) {
        map.planning_dis_map[tx][ty] = map.planning_dis_map[x][y] + dis[i];
        obj_que.push(make_pair(tx, ty));
      }
    }
  }
}

void MapManager::getAccessibleMap() {
  const int dx[] = {0, 0, -1, 1};
  const int dy[] = {-1, 1, 0, 0};

  memset(map.accessible_map, false, sizeof(map.accessible_map));
  queue<pair<int, int>> obj_que;
  map.accessible_map[CAR_CEN_ROW][CAR_CEN_COL] = true;
  obj_que.push(make_pair(CAR_CEN_ROW, CAR_CEN_COL));

  bool visited[MAX_ROW][MAX_COL];
  memset(visited, false, sizeof(visited));
  while (!obj_que.empty()) {
    const int x = obj_que.front().first;
    const int y = obj_que.front().second;
    obj_que.pop();
    for (int i = 0; i < 4; ++i) {
      const int tx = x + dx[i], ty = y + dy[i];
      if (!visited[tx][ty] && Point2d(tx, ty).in_map() &&
          map.planning_dis_map[tx][ty] * GRID_RESOLUTION >
              COLLISION_CIRCLE_SMALL_R) {
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
  if (map.lane_list.detected) {
    map.v_line_list.push_back(
        map.lane_list.lane_list.front().right_line.points);
    for (const auto& lane : map.lane_list.lane_list) {
      map.v_line_list.push_back(lane.left_line.points);
    }
  }
  //通过边界车道线矫正参考路
  double     ref_offset_dis = 0;
  HDMapPoint first_p;
  for (auto& hd_p : map.ref_path) {
    if (hd_p.s >= 0) {
      first_p = hd_p;
      break;
    }
  }
  if (!map.v_line_list.empty()) {
    //优先以左边界线矫正
    if (map.v_line_list.back().front().type == LineType::BOUNDARY) {
      //计算参考路所指最左侧车道线的点
      double most_left_dis =
          first_p.lane_width * (first_p.lane_num - first_p.lane_seq + 0.5);
      Pose most_left_p = first_p.getLateralPose(most_left_dis);
      //计算最左侧车道线点和视觉检测边界线的偏移距离
      LinePoint v_1_p = map.v_line_list.back()[0];
      LinePoint v_2_p = map.v_line_list.back()[1];
      Point2d   vec_a = most_left_p - v_1_p;
      Point2d   vec_b = v_2_p - v_1_p;
      ref_offset_dis  = vec_a.cross(vec_b) / vec_b.len();
    }
    //其次以右边界矫正
    else if (map.v_line_list.front().front().type == LineType::BOUNDARY) {
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
      if (map.v_line_list.size() == first_p.lane_num + 1) {
        double most_left_dis =
            first_p.lane_width * (first_p.lane_num - first_p.lane_seq + 0.5);
        Pose most_left_p = first_p.getLateralPose(most_left_dis);
        //计算最左侧车道线点和视觉检测边界线的偏移距离
        LinePoint v_1_p = map.v_line_list.back()[0];
        LinePoint v_2_p = map.v_line_list.back()[1];
        Point2d   vec_a = most_left_p - v_1_p;
        Point2d   vec_b = v_2_p - v_1_p;
        ref_offset_dis  = vec_a.cross(vec_b) / vec_b.len();
      }
      //如果视觉车道线少于地图车道线,直接右边界匹配效果好
      else if (map.v_line_list.size() < first_p.lane_num + 1) {
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
    if (ref_offset_dis != 0) {
      for (auto& p : map.ref_path) {
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
  return ceil((dis * GRID_RESOLUTION - p_ref.lane_width / 2) /
              p_ref.lane_width) +
         p_ref.lane_seq;
}

vector<Pose> MapManager::getExplorationTargets() {
  vector<Pose> targets;
  const double dis[] = {5, 8, 11, 14, 17};
  if (this->map.nav_info.detected && !(this->map.forward_ref_path.empty())) {
    int idx = 0;
    for (auto& point : this->map.forward_ref_path) {
      if (point.s >= dis[idx]) {
        idx++;
        for (int i = 1;; ++i) {
          Pose a = point.getLateralPose(-0.5 * i);
          Pose b = point.getLateralPose(0.5 * i);
          if (!a.in_map() && !b.in_map()) break;
          if (a.in_map() && !collision(a, this->map.planning_dis_map) &&
              this->map.accessible_map[int(a.x)][int(a.y)]) {
            targets.push_back(a);
            break;
          }
          if (b.in_map() && !collision(b, this->map.planning_dis_map) &&
              this->map.accessible_map[int(b.x)][int(b.y)]) {
            targets.push_back(b);
            break;
          }
        }
      }
      if (idx >= sizeof(dis) / sizeof(double)) break;
    }
  } else if (!(this->map.v_line_list.empty())) {
  } else {
    ;
  }
  return targets;
}

void MapManager::laneMatch() {
  //视觉检测到车道线匹配
  //若视觉检测和地图车道线数量一样
  if (map.lane_line_list.size() == map.v_line_list.size()) {
    for (int i = 0; i < map.lane_line_list.size(); ++i) {
      int shortest_index =
          shortestPointIndex(map.nav_info.car_pose, map.lane_line_list[i]);
      LinePoint lp     = map.lane_line_list[i][shortest_index];
      LinePoint vp1    = map.v_line_list[i].front();
      LinePoint vp2    = map.v_line_list[i][1];
      Point2d   vec_a  = vp2 - vp1;
      Point2d   vec_b  = lp - vp1;
      double    offset = (vec_b.cross(vec_a) / vec_a.len()) * GRID_RESOLUTION;
      Point2d   off_p;
      for (int j = 0; j + 1 < map.lane_line_list[i].size(); ++j) {
        Point2d vec_d = map.lane_line_list[i][j + 1] - map.lane_line_list[i][j];
        off_p         = offsetPoint(vec_d, offset);
        map.lane_line_list[i][j].x += off_p.x;
        map.lane_line_list[i][j].y += off_p.y;
      }
      map.lane_line_list[i][map.lane_line_list.size() - 1].x += off_p.x;
      map.lane_line_list[i][map.lane_line_list.size() - 1].y += off_p.y;
    }
  } else {
    //如果车道线数量不一样，左边界对齐或者右边界对齐
    // TODO
  }
  laneLineInterpolation();
}

void MapManager::maintainParkingSpots() {
  for (const auto& spot : map.parking_lot_list.parking_lot_list) {
    Point2d center_point = (spot.left_back + spot.left_front + spot.right_back +
                            spot.right_front) *
                           0.25;
    double ang       = (spot.right_front - spot.right_back).getRad();
    Pose   spot_pose = Pose(center_point.x, center_point.y, ang);
    spot_pose.updateGlobalCoordinate(map.nav_info.car_pose);
    double min_dis = 999999999;
    int    min_idx = -1;
    for (int i = 0; i < map.parking_spots.size(); ++i) {
      double dx = spot_pose.utm_position.utm_x -
                  map.parking_spots[i].utm_position.utm_x;
      double dy = spot_pose.utm_position.utm_y -
                  map.parking_spots[i].utm_position.utm_y;
      double dis = hypot(dx, dy);
      if (dis < min_dis) {
        min_dis = dis;
        min_idx = i;
      }
    }
    if (min_dis <= 5) {
      map.parking_spots[min_idx] = spot_pose;
    } else {
      map.parking_spots.push_back(spot_pose);
    }
  }
}

std::vector<Pose> MapManager::getParkingSpotTarget() {
  vector<Pose> targets;
  for (auto spot : map.parking_spots) {
    spot.updateLocalCoordinate(map.nav_info.car_pose);
    if (spot.in_map() && map.accessible_map[int(spot.x)][int(spot.y)]) {
      targets.push_back(spot);
      return targets;
    }
  }
  return targets;
}

Pose MapManager::getTemporaryParkingTarget() {
  Pose target;
  auto current_tasks = this->getCurrentTasks();
  if (current_tasks.empty()) {
    Task parking_task = this->getParkingTask();
    if (parking_task.task_points.empty()) return target;
    current_tasks.push_back(parking_task);
  }
  for (const auto& spot : current_tasks.back().task_points) {
    Pose p;
    p.utm_position = spot;
    p.updateLocalCoordinate(map.nav_info.car_pose);
    if (p.in_map() && map.accessible_map[int(p.x)][int(p.y)]) {
      return p;
    }
  }
  return target;
}

vector<Pose> MapManager::getTaskTarget() {
  vector<Pose> targets;
  auto         current_tasks = this->getCurrentTasks();
  for (auto task_point : current_tasks.back().task_points) {
    Pose task_pose;
    task_pose.utm_position = task_point;
    task_pose.updateLocalCoordinate(map.nav_info.car_pose);
    if (task_pose.in_map() &&
        map.accessible_map[int(task_pose.x)][int(task_pose.y)]) {
      targets.push_back(task_pose);
      return targets;
    }
  }
  return targets;
}

void MapManager::laneLineInterpolation() {
  for (auto& line : map.lane_line_list) {
    lineInterpolation<LinePoint>(line);
  }
  for (auto& line : map.lane_center_list) {
    lineInterpolation<Point2d>(line);
  }
}

void MapManager::visualization() {
  MessageManager&   msgm = MessageManager::getInstance();
  visVISUALIZATION& vis  = msgm.visualization;
  msgm.setTextInfo();
  // reference path
  vis.reference_path.clear();
  vis.reference_path_size = map.forward_ref_path.size();
  for (const auto& p : map.forward_ref_path) {
    visPoint vp;
    vp.x = p.x;
    vp.y = p.y;
    vis.reference_path.push_back(vp);
  }
  // planning map
  memcpy(vis.safe_map, map.planning_dis_map, sizeof(map.planning_dis_map));
  // lanes
  vis.lanes.clear();
  vis.lanes_size = 1;
  visLaneLine vll;
  vll.lane_line_points_size = 0;
  for (const auto& ref_p : map.ref_path) {
    for (int i = 0; i < ref_p.lane_num + 1; ++i) {
      const auto& lane_p =
          ref_p.getLateralPose((i - ref_p.lane_seq + 0.5) * ref_p.lane_width);
      visPoint vp;
      vp.x = lane_p.x;
      vp.y = lane_p.y;
      vll.lane_line_points_size += 1;
      vll.lane_line_points.push_back(vp);
    }
  }
  vis.lanes.push_back(vll);
  // speed planner
  msgm.publishVisualization();
}

vector<Pose> MapManager::getStartMaintainedPath() {
  std::vector<Pose> path =
      DecisionContext::getInstance().getMaintainedPath(map.nav_info);
  if (path.empty()) return path;
  int safe_size = path.size();
  for (int i = 0; i < path.size(); ++i) {
    const auto& p = path[i];
    if (map.planning_dis_map[(int)p.x][(int)p.y] * GRID_RESOLUTION <
        COLLISION_CIRCLE_SMALL_R) {
      safe_size = i + 1;
      break;
    }
  }
  // get maintain s by car speed
  double maintain_s   = 2 * map.nav_info.current_speed;
  int    maintain_idx = 0;
  for (const auto& p : path) {
    maintain_idx++;
    if (p.s >= maintain_s) break;
  }
  // chose the shorter one
  safe_size = std::min(safe_size, maintain_idx);
  path.resize(safe_size);
  if (path.size() <= 2 ||
      (!path.empty() &&
       point2PointSqrDis(path.front(), map.nav_info.car_pose) > 5)) {
    if (point2PointSqrDis(path.front(), map.nav_info.car_pose) > 5) {
      LOG(WARNING) << "start maintain path clear() !";
    }
    path.clear();
  }
  return path;
}

vector<Pose> MapManager::getMaintainedPath(const NavInfo& nav_info) {
  maintained_path_mutex.lock_shared();
  auto path = maintained_path;
  maintained_path_mutex.unlock_shared();
  for (auto& p : path) {
    p.updateLocalCoordinate(nav_info.car_pose);
  }
  if (path.empty()) return path;
  int    shortest_index = shortestPointIndex(nav_info.car_pose, path);
  double base_s         = path[shortest_index].s;
  for (auto& p : path) {
    p.s -= base_s;
  }
  vector<Pose> res;
  if (path[shortest_index].backward) {
    for (auto p : path) {
      if (p.s < 0) continue;
      if (p.in_map() && p.backward)
        res.push_back(p);
      else
        break;
    }
  } else {
    for (auto p : path) {
      if (p.s < 0) continue;
      if (p.in_map() && !p.backward)
        res.push_back(p);
      else
        break;
    }
  }
  /*
  if (res.size() < path.size() && !res.empty() && res.back().s <= 3) {
    res.clear();
    bool back_ward    = path[shortest_index].backward;
    int  second_index = -1;
    for (int k = 0; k < path.size(); ++k) {
      Pose p = path[k];
      if (p.s < 0) continue;
      if (p.backward == back_ward) {
        continue;
      }
      second_index = k;
      break;
    }
    for (int k = second_index; k < path.size(); ++k) {
      Pose p = path[k];
      if (p.s < 0) continue;
      if (p.backward != back_ward) {
        res.push_back(p);
      } else
        break;
    }
  }
  */
  return res;
}

void MapManager::getSpeedMaintainedPath(NavInfo& nav_info) {
  vector<Pose> result_path = getMaintainedPath(nav_info);
  map.speed_maintained_path.path.clear();
  map.speed_maintained_path.st_boundaries.clear();
  if (result_path.empty()) return;
  for (auto& p : result_path) {
    p.v = 0;
    p.t = inf;
  }

  vector<pair<double, double>> speed_limits;
  speed_limits.reserve(result_path.size());
  // conversion
  for (auto& point : result_path) {
    double max_speed = getCurrentMapSpeed();
    if (point.backward) {
      point.s   = fabs(point.s);
      max_speed = 2;
      point.ang = PI + point.ang;
    }
  }

  result_path.front().v      = fabs(nav_info.current_speed);
  bool add_collision_dynamic = false;
  for (const auto& p : result_path) {
    if (!add_collision_dynamic && collision(p, map.lidar_dis_map)) {
      add_collision_dynamic = true;
      DynamicObj dummy_obj;
      dummy_obj.width  = 1.5;
      dummy_obj.length = 3;
      dummy_obj.path.emplace_back(p.x, p.y, p.ang, 0, 0, 0);
      map.dynamic_obj_list.dynamic_obj_list.push_back(dummy_obj);
    }
    double max_speed = getCurrentMapSpeed();
    if (p.backward) max_speed = 2;
    speed_limits.emplace_back(p.s,
                              min(max_speed, max_velocity_for_curvature(p.k)));
  }
  map.speed_maintained_path = SpeedOptimizer::RunSpeedOptimizer(
      map.dynamic_obj_list.dynamic_obj_list, result_path, speed_limits,
      result_path.back().s, nav_info.current_speed);

  // anti-conversion
  for (auto& point : map.speed_maintained_path.path) {
    if (point.backward) {
      point.ang = NormalizeAngle(point.ang - PI);
      point.v   = -point.v;
      point.a   = -point.a;
    }
  }
}

void MapManager::dynamicDecision(const DynamicBlockType dynamic_block_type) {
  memset(map.dynamic_obs_map, 0, sizeof(map.dynamic_obs_map));
  const auto add_obs_between = [&](const Point2d& start, const Point2d& end) {
    Point2d direction = (end - start).getDirection();
    for (int i = 0; i < (end - start).len(); ++i) {
      Point2d now = start + direction * i;
      if (now.in_map()) map.dynamic_obs_map[(int)now.x][(int)now.y] = 1;
    }
  };
  // block the dynamic speed under 0.1m/s
  for (const auto& obstacle : map.dynamic_obj_list.dynamic_obj_list) {
    // don't block low speed dynamic obj in NO_BLOCK mode
    if (dynamic_block_type == MapManager::DynamicBlockType::NO_BLOCK) {
      break;
    }
    if (obstacle.path.size() < 2) continue;
    const double speed = (obstacle.path[1] - obstacle.path[0]).len();
    if (speed > 0.1) continue;
    for (int i = 0; i < obstacle.corners.size(); ++i) {
      const auto& start = obstacle.corners[i];
      const auto& end =
          obstacle.corners[(i + 1) % (int)obstacle.corners.size()];
      add_obs_between(start, end);
    }
  }
}

void MapManager::laneBlockDecision() {
  memset(map.lane_block_map, 0, sizeof(map.lane_block_map));
  const auto add_obs_between = [&](const Pose& start, const Pose& end) {
    const auto direction = (end - start).getDirection();
    for (int i = 0; i < (end - start).len(); ++i) {
      const auto now = start + direction * i;
      if (now.in_map()) map.lane_block_map[(int)now.x][(int)now.y] = 1;
    }
  };
  // block uncorrect lane in intersection
  for (const auto& p : map.ref_path) {
    // if it's not intersection solid or there is only 1 lane, don't block
    if (p.mode != HDMapMode::INTERSECTION_SOLID || p.lane_num < 2) continue;
    double block_start_point_dis = 0;
    double block_end_point_dis   = 0;
    // if there are 2 lane
    if (p.lane_num == 2) {
      if (p.direction == RoadDirection::RIGHT ||
          p.direction == RoadDirection::STRAIGHT) {
        // if we'll turn right or go straight, block the left lane
        block_start_point_dis = (p.lane_num - p.lane_seq - 0.5) * p.lane_width;
        block_end_point_dis   = block_start_point_dis + p.lane_width;
      } else {
        // if we'll turn left, block the right lane
        block_start_point_dis = (1 - p.lane_seq - 0.5) * p.lane_width;
        block_end_point_dis   = block_start_point_dis + p.lane_width;
      }
    } else {
      // if the lanes are more than 2
      if (p.direction == RoadDirection::RIGHT) {
        // if we'll turn right, block all the lane except the right one
        block_start_point_dis = (1 - p.lane_seq + 0.5) * p.lane_width;
        block_end_point_dis   = (p.lane_num - p.lane_seq + 0.5) * p.lane_width;
      } else if (p.direction == RoadDirection::LEFT) {
        // if we'll turn left, block all the lane except the left one
        block_start_point_dis = (1 - p.lane_seq - 0.5) * p.lane_width;
        block_end_point_dis   = (p.lane_num - p.lane_seq - 0.5) * p.lane_width;
      } else {
        // if we'll go straight, block the left and right lanes
        const double right_start_dis = (1 - p.lane_seq - 0.5) * p.lane_width;
        const double right_end_dis   = right_start_dis + p.lane_width;
        const double left_start_dis =
            (p.lane_num - p.lane_seq - 0.5) * p.lane_width;
        const double left_end_dis = left_start_dis + p.lane_width;
        add_obs_between(p.getLateralPose(right_start_dis),
                        p.getLateralPose(right_end_dis));
        add_obs_between(p.getLateralPose(left_start_dis),
                        p.getLateralPose(left_end_dis));
      }
    }
    if (block_end_point_dis != block_start_point_dis) {
      add_obs_between(p.getLateralPose(block_start_point_dis),
                      p.getLateralPose(block_end_point_dis));
    }
  }
}

void MapManager::maintainPath(const NavInfo&      nav_info,
                              const vector<Pose>& path) {
  if (path.empty()) return;
  maintained_path_mutex.lock();
  maintained_path.clear();
  maintained_path.reserve(path.size());
  for (const auto& p : path) {
    // the point have been updated in path planner
    // p.updateGlobalCoordinate(nav_info.car_pose);
    maintained_path.push_back(p);
  }
  maintained_path_mutex.unlock();
}

void MapManager::selectBestPath(const vector<SpeedPath>& paths) {
  if (!paths.empty())
    map.best_path = paths.front();
  else
    map.best_path = SpeedPath();
}

bool MapManager::allowParking(const Pose&                    parking_spot,
                              const std::vector<HDMapPoint>& ref_path) {
  if (parking_spot.x == 0 && parking_spot.y == 0 && parking_spot.ang == 0) {
    return false;
  }
  if (ref_path.empty()) return true;
  double min_dis = std::numeric_limits<double>::max();
  Pose   near_ref_p;
  for (const auto& ref_p : ref_path) {
    const double dis = (parking_spot - ref_p).sqrLen();
    if (dis < min_dis) {
      min_dis    = dis;
      near_ref_p = ref_p;
    }
  }
  // if parking spot is low than 20m in reference path, parking
  if (near_ref_p.s < 20) return true;
  return false;
}

const std::vector<HDMapPoint> MapManager::getLaneCenterDecision(
    const Map& decision_map) {
  auto ref_path = decision_map.ref_path;

  // resize the ref_path to get the first one path
  auto new_size = ref_path.size();
  for (int i = 0; i < ref_path.size(); ++i) {
    const auto& p = ref_path[i];
    if (p.s < 0) continue;
    if (p.s >= 120 || int(p.x) <= 2 || int(p.x) > MAX_ROW - 2 ||
        int(p.y) <= 1 || int(p.y) >= MAX_COL - 2) {
      new_size = i + 1;
      break;
    }
  }
  ref_path.resize(new_size);
  // get the lane center for each ref path p
  for (auto& p : ref_path) {
    if (!p.neighbors.empty() || p.mode == CHANGE) continue;
    for (int i = 0; i < p.lane_num; ++i) {
      const auto& neighbor_p =
          p.getLateralPose(p.lane_width * (i - p.lane_seq + 1));
      p.neighbors.emplace_back(neighbor_p.x, neighbor_p.y, true, 0.0, false);
    }
    // maybe we could add a oppsite lane center with no priority
  }
  const auto& clash_with_dynamic = [&](const double x, const double y) {
    try {
      for (const auto& obj : decision_map.dynamic_obj_list.dynamic_obj_list) {
        Point2d o_p(x, y);
        /*
        double  last_cross_val =
            (obj.corners.back() - o_p).cross(obj.corners.front() - o_p);
        for (int i = 0; i + 1 < obj.corners.size(); ++i) {
          const auto& p0        = obj.corners[i];
          const auto& p1        = obj.corners[i + 1];
          const auto& cross_val = (p0 - o_p).cross(p1 - o_p);
          if (cross_val * last_cross_val <= 0) return false;
          last_cross_val = cross_val;
        }
        */
        for (const auto& p : obj.corners) {
          if ((p - o_p).len() * GRID_RESOLUTION < COLLISION_CIRCLE_SMALL_R)
            return true;
        }
      }
      return false;
    } catch (const std::exception& e) {
      LOG(FATAL) << "maby the dynamic obj have no corners!";
    }
  };
  // set the center line priority to avoid bostacles
  HDMapPoint       last_ref_p;
  constexpr double max_effect_dis = 15;  // m
  // backward check the path
  for (auto it = ref_path.rbegin(); it != ref_path.rend(); it++) {
    auto& ref_p = *it;
    if (ref_p.s < 0) break;
    // LOG(WARNING) << "------ s=" << ref_p.s << " -------";
    for (int i = 0; i < ref_p.neighbors.size(); ++i) {
      auto& center_p = ref_p.neighbors[i];
      // find the closest pre lane center point and set priority by pre
      double          min_dis = std::numeric_limits<double>::max();
      bool            have_near_center_point = false;
      LaneCenterPoint pre_nearest_center_point;
      for (const auto& last_center_p : last_ref_p.neighbors) {
        const double dis = (last_center_p - center_p).len() * GRID_RESOLUTION;
        if (dis < min_dis) {
          min_dis                  = dis;
          have_near_center_point   = true;
          pre_nearest_center_point = last_center_p;
        }
      }
      // decide the priority
      // if the center have priority but crashed
      // LOG(INFO) << "collision with map:"
      //           << collision(center_p.x, center_p.y,
      //                        decision_map.planning_dis_map);
      // LOG(INFO) << "collision with dynamic:"
      //           << clash_with_dynamic(center_p.x, center_p.y);
      const bool clash_with_static =
          collision(center_p.x, center_p.y, decision_map.planning_dis_map);
      if (clash_with_static || clash_with_dynamic(center_p.x, center_p.y)) {
        center_p.have_priority                = false;
        center_p.accumulate_dis_with_priority = 0.0;
        center_p.accumulate_by_static_obs     = clash_with_static;
        if (have_near_center_point &&
            pre_nearest_center_point.accumulate_by_static_obs) {
          center_p.have_priority = pre_nearest_center_point.have_priority;
          center_p.accumulate_dis_with_priority =
              pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
          center_p.accumulate_by_static_obs =
              pre_nearest_center_point.accumulate_by_static_obs;
        }
      } else if (have_near_center_point) {
        if (pre_nearest_center_point.accumulate_by_static_obs) {
          center_p.have_priority = pre_nearest_center_point.have_priority;
          center_p.accumulate_dis_with_priority =
              pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
          center_p.accumulate_by_static_obs =
              pre_nearest_center_point.accumulate_by_static_obs;
        } else if (pre_nearest_center_point.accumulate_dis_with_priority <
                   max_effect_dis) {
          center_p.have_priority = pre_nearest_center_point.have_priority;
          center_p.accumulate_dis_with_priority =
              pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
        }
      }
      // LOG(INFO) << center_p;
    }
    // check if all the lane center have no priority
    bool all_have_no_priority = !ref_p.neighbors.empty();
    for (const auto& cp : ref_p.neighbors) {
      if (cp.have_priority) {
        all_have_no_priority = false;
        break;
      }
    }
    if (all_have_no_priority) {
      // condider with the oppsite lane
      if (!(ref_p.block_type & 0x2)) {
        // if the opposite lane can be borrowed
        const auto& oppsite_p = ref_p.getLateralPose(
            (ref_p.lane_num + 1 - ref_p.lane_seq) * ref_p.lane_width);
        if (collision(oppsite_p.x, oppsite_p.y,
                      decision_map.planning_dis_map) ||
            clash_with_dynamic(oppsite_p.x, oppsite_p.y)) {
        } else {
          ref_p.neighbors.emplace_back(oppsite_p.x, oppsite_p.y, true, 0.0,
                                       false);
          all_have_no_priority = false;
        }
      }
    }
    if (all_have_no_priority) {
      // consider with lane center only clashed with dynamic
      for (auto& center_p : ref_p.neighbors) {
        if (!collision(center_p.x, center_p.y, decision_map.planning_dis_map)) {
          center_p.have_priority = true;
          all_have_no_priority   = false;
        }
      }
    }
    if (all_have_no_priority) {
      // TODO:if still have no priority, consider the road outside?
      // set all is ok
      for (auto& center_p : ref_p.neighbors) {
        center_p.have_priority = true;
        all_have_no_priority   = false;
      }
    }
    last_ref_p = ref_p;
  }
  // forward check the path
  last_ref_p.neighbors.clear();
  constexpr double max_forward_effect_dis = 100;  // m
  for (auto it = ref_path.begin(); it != ref_path.end(); it++) {
    auto& ref_p = *it;
    if (ref_p.s < 0) continue;
    // LOG(WARNING) << "------ s=" << ref_p.s << " -------";
    for (int i = 0; i < ref_p.neighbors.size(); ++i) {
      auto& center_p = ref_p.neighbors[i];
      // find the closest pre lane center point and set priority by pre
      double          min_dis = std::numeric_limits<double>::max();
      bool            have_near_center_point = false;
      LaneCenterPoint pre_nearest_center_point;
      for (const auto& last_center_p : last_ref_p.neighbors) {
        const double dis = (last_center_p - center_p).len() * GRID_RESOLUTION;
        if (dis < min_dis) {
          min_dis                  = dis;
          have_near_center_point   = true;
          pre_nearest_center_point = last_center_p;
        }
      }
      // decide the priority
      // if the center have priority but crashed
      // LOG(INFO) << "collision with map:"
      //           << collision(center_p.x, center_p.y,
      //                        decision_map.planning_dis_map);
      // LOG(INFO) << "collision with dynamic:"
      //           << clash_with_dynamic(center_p.x, center_p.y);
      if (collision(center_p.x, center_p.y, decision_map.planning_dis_map) ||
          clash_with_dynamic(center_p.x, center_p.y)) {
        center_p.have_priority                = false;
        center_p.accumulate_dis_with_priority = 0.0;
      } else if (have_near_center_point) {
        if (pre_nearest_center_point.accumulate_dis_with_priority <
            max_forward_effect_dis) {
          center_p.have_priority = pre_nearest_center_point.have_priority;
          center_p.accumulate_dis_with_priority =
              pre_nearest_center_point.accumulate_dis_with_priority + min_dis;
        }
      }
      // LOG(INFO) << center_p;
    }
    // check if all the lane center have no priority
    bool all_have_no_priority = !ref_p.neighbors.empty();
    for (const auto& cp : ref_p.neighbors) {
      if (cp.have_priority) {
        all_have_no_priority = false;
        break;
      }
    }
    if (all_have_no_priority) {
      // consider with lane center only clashed with dynamic
      for (auto& center_p : ref_p.neighbors) {
        if (!collision(center_p.x, center_p.y, decision_map.planning_dis_map)) {
          center_p.have_priority = true;
          all_have_no_priority   = false;
        }
      }
    }
    if (all_have_no_priority) {
      // TODO:if still have no priority, consider the road outside?
      // set all is ok
      for (auto& center_p : ref_p.neighbors) {
        center_p.have_priority = true;
        all_have_no_priority   = false;
      }
    }
    last_ref_p = ref_p;
    // LOG(WARNING) << "---at ref_p:" << ref_p.s << "---";
    // for (const auto& p : ref_p.neighbors) {
    //   LOG(INFO) << p;
    // }
  }
  // send to visualization center line offset
  MessageManager::getInstance().setPriorityLane(ref_path);
  return ref_path;
}

// This uses the ray-casting algorithm to decide whether the point is inside
// the given polygon. See
// https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
bool MapManager::pnbox(const Pose& point, const vector<Pose>& box) {
  if (box.size() < 3) return false;
  double x = point.x;
  double y = point.y;
  // If we never cross any lines we're inside.
  bool inside = false;
  // Loop through all the edges.
  for (int i = 0; i < box.size(); ++i) {
    // i is the index of the first vertex, j is the next one.
    // The original code uses a too-clever trick for this.
    int j = (i + 1) % box.size();
    // The vertices of the edge we are checking.
    double xp0 = box[i].x;
    double yp0 = box[i].y;
    double xp1 = box[j].x;
    double yp1 = box[j].y;
    // Check whether the edge intersects a line from (-inf,y) to (x,y).
    // First check if the line crosses the horizontal line at y in either
    // direction.
    if ((yp0 <= y) && (yp1 > y) || (yp1 <= y) && (yp0 > y)) {
      // If so, get the point where it crosses that line. This is a simple
      // solution to a linear equation. Note that we can't get a division by
      // zero here - if yp1 == yp0 then the above if be false.
      double cross = (xp1 - xp0) * (y - yp0) / (yp1 - yp0) + xp0;
      // Finally check if it crosses to the left of our test point. You could
      // equally do right and it should give the same result.
      if (cross < x) inside = !inside;
    }
  }
  if (inside) return true;
  return false;
}

void MapManager::predDynamicObjTraj() {
  dynamic_obj_mutex.lock();
  // 1.initialize lattice planner
  LatticePlanner     lp;
  vector<HDMapPoint> ref_path = getRefPath();
  // 2.only consider vehicles which are on the road with the same direction
  for (DynamicObj& dynamic_obj : map.dynamic_obj_list.dynamic_obj_list) {
    if (dynamic_obj.path.size() <= 1) continue;
    vector<Pose> ori_path     = dynamic_obj.path;
    Pose         current_pose = ori_path[0];
    double       min_id       = -1;
    double       min_dis      = std::numeric_limits<double>::max();
    double       dis          = -1;
    for (int i = 0; i < ref_path.size(); ++i) {
      dis = std::pow(ref_path[i].x - current_pose.x, 2) +
            std::pow(ref_path[i].y - current_pose.y, 2);
      if (dis < min_dis) {
        min_dis = dis;
        min_id  = i;
      }
    }
    if (min_id == -1) continue;
    Pose matched_pose = ref_path[min_id];
    if (fabs(matched_pose.ang - current_pose.ang) >= PI / 3) continue;

    Point2d v1(std::cos(matched_pose.ang), std::sin(matched_pose.ang));
    Point2d v2(current_pose.x - matched_pose.x,
               current_pose.y - matched_pose.y);
    double  cross = v1.cross(v2);
    double  left_bound, right_bound;
  }
  // 3.lane-keep or lane change decision
  // 4.predict trajectory

  dynamic_obj_mutex.unlock();
  return;
}

MapManager* MapManager::instance = new MapManager;
}  // namespace TiEV
