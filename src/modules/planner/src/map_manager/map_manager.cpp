#include "map_manager.h"

#include <fstream>
#include <iostream>
#include <queue>
#include <shared_mutex>

#include "collision_check.h"
#include "tiev_utils.h"
#include "tievlog.h"

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
  message_manager->getRainSignal(map.rain_signal);
  message_manager->getMap(map.lidar);
  //-------
  handleLidarMap();
  // getSpeedMaintainedPath(map.nav_info);
}

double MapManager::getSpeedBySpeedMode(int speed_mode) {
  Config* cfg = Config::getInstance();
  if (cfg->control_mode == ControlMode::PlanningWithDebugMode ||
      cfg->control_mode == ControlMode::TrakingWithDebugMode)
    speed_mode = cfg->debug_speed_mode;
  switch (speed_mode) {
    case 0:
      return cfg->back_speed / 3.6;
    case 1:
      return cfg->stop_speed / 3.6;
    case 2:
      return cfg->very_low_speed / 3.6;
    case 3:
      return cfg->low_speed / 3.6;
    case 4:
      return cfg->mid_speed / 3.6;
    case 5:
      return cfg->high_speed / 3.6;
    case 6:
      return cfg->very_high_speed / 3.6;
    default:
      return cfg->stop_speed / 3.6;
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
    HDMapPoint point(utm_x, utm_y, heading, curve, (HDMapMode)mode,
                     (HDMapSpeed)speed_mode, (HDMapEvent)event,
                     (BlockType)block_type, lane_num, lane_seq, lane_width);
    global_path.push_back(point);
  }
  filtPoints();
  setGlobalPathDirection();
  std::cout << "Map file already loaded!Enjoy!" << endl;
}

vector<Task> MapManager::getCurrentTasks() {
  task_points_mutex.lock_shared();
  vector<Task> res = this->map.current_task_points;
  task_points_mutex.unlock_shared();
  return res;
}

Task MapManager::getParkingTask() {
  parking_task_mutex.lock_shared();
  Task res = this->map.parking_task;
  parking_task_mutex.unlock_shared();
  return res;
}
void MapManager::popCurrentTask() {
  if (this->map.current_task_points.empty()) return;
  task_points_mutex.lock();
  this->map.current_task_points.pop_back();
  task_points_mutex.unlock();
}

void MapManager::pushCurrentTask(const Task& next_task) {
  task_points_mutex.lock_shared();
  this->map.current_task_points.push_back(next_task);
  task_points_mutex.unlock_shared();
}

void MapManager::clearTask() {
  task_points_mutex.lock();
  this->map.current_task_points.clear();
  task_points_mutex.unlock();
}

void MapManager::setGlobalPath(const vector<HDMapPoint>& new_global_path) {
  global_path_mutex.lock();
  this->global_path = new_global_path;
  filtPoints();
  setGlobalPathDirection();
  global_path_nearest_idx = -1;
  global_path_mutex.unlock();
}

void MapManager::runRouting(int interval, bool blocked) {
  while (true) {
    time_t current_time = getTimeStamp();
    if (current_time - global_path_update_time > interval) {
      Routing*           routing = Routing::getInstance();
      vector<HDMapPoint> tmp_global_path;
      task_points_mutex.lock_shared();
      // int cost = routing->findReferenceRoad(tmp_global_path,
      //                                       map.current_task_points,
      //                                       blocked);
      task_points_mutex.unlock_shared();
      if (true) {  // cost <
                   // &&sustitude(global_path,
                   // tmp_global_path)) {
                   // //
                   // TODO: How to use the
                   // cost?
        global_path_mutex.lock();
        this->global_path = tmp_global_path;
        filtPoints();
        setGlobalPathDirection();
        global_path_nearest_idx = -1;
        global_path_mutex.unlock();
      }
      global_path_update_time = current_time;
    }
  }
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

int MapManager::getGlobalPathNearestIndex(int begin, int end) const {
  UtmPosition vehicle_position = map.nav_info.car_pose.utm_position;
  double      min_dis          = 1e10;
  int         min_idx          = -1;
  double      dx, dy, da, dis;
  for (int i = max(0, begin); i < min(int(global_path.size()), end + 1); ++i) {
    dx  = vehicle_position.utm_x - global_path[i].utm_position.utm_x;
    dy  = vehicle_position.utm_y - global_path[i].utm_position.utm_y;
    dis = sqrt(dx * dx + dy * dy);
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
      current_idx_in_ref_path = map.ref_path.size() - 1;
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
  getLaneLineList();
  // laneMatch();
  // getBoundaryLine();
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
    if (p.event != HDMapEvent::ENTRY_INTERSECTION) continue;
    int        x = p.x;
    int        y = p.y;
    DynamicObj dummy_obj;
    dummy_obj.width  = 1.5;
    dummy_obj.length = 10;
    dummy_obj.path.emplace_back(x, y, PI, 0, 0, 0);
    map.dynamic_obj_list.dynamic_obj_list.emplace_back(dummy_obj);
    break;
  }
}

bool MapManager::carInRoad() {
  if (map.boundary_line.empty()) return false;
  if (point2LineDis(map.nav_info.car_pose, map.boundary_line.front()) <= 0)
    return false;
  if (map.boundary_line.size() < 2) return true;
  if (point2LineDis(map.nav_info.car_pose, map.boundary_line.back()) >= 0)
    return false;
  return true;
}

void MapManager::updatePlanningMap(LaneLineBlockType lane_line_block_type,
                                   bool              history) {
  memset(map.line_block_map, 0, sizeof(map.line_block_map));
  if (lane_line_block_type == LaneLineBlockType::NO_BLOCK) {
    if (map.boundary_line.size() > 1) {
      const auto& line = map.boundary_line.back();
      for (const auto& p : line) {
        if (p.type == LineType::BOUNDARY)
          map.line_block_map[int(p.x)][int(p.y)] = 1;
      }
    }
  } else {
    // block uncrrect lane in intersection
    for (const auto& p : map.ref_path) {
      if (getCurrentMapMode() == HDMapMode::INTERSECTION_SOLID) break;
      if (p.mode == HDMapMode::INTERSECTION_SOLID) {
        if (p.lane_num < 2) {
          continue;
        } else if (p.lane_num == 2) {
          double base_dis = 0;
          if (p.direction == RoadDirection::RIGHT ||
              p.direction == RoadDirection::STRAIGHT)
            base_dis = (p.lane_num - p.lane_seq - 0.5) * p.lane_width;
          else
            base_dis = (p.lane_num - p.lane_seq - 1.5) * p.lane_width;
          int k = p.lane_width / 0.5;
          for (int i = 1; i < k; ++i) {
            Pose block_p = p.getLateralPose(base_dis + i * 0.5);
            map.line_block_map[int(block_p.x)][int(block_p.y)] = 1;
          }
        } else {
          double base_dis     = 0;
          int    start_lane_i = 1;
          int    end_lane_i   = 0;
          if (p.direction == RoadDirection::RIGHT) {
            start_lane_i = 2;
            end_lane_i   = p.lane_num;
          } else if (p.direction == RoadDirection::LEFT) {
            start_lane_i = 1;
            end_lane_i   = p.lane_num - 1;
          }
          for (int lane_i = start_lane_i; lane_i <= end_lane_i; ++lane_i) {
            base_dis = (lane_i - p.lane_seq - 0.5) * p.lane_width;
            int k    = p.lane_width / 0.5;
            for (int i = 1; i < k; ++i) {
              Pose block_p = p.getLateralPose(base_dis + i * 0.5);
              map.line_block_map[int(block_p.x)][int(block_p.y)] = 1;
            }
          }
          if (p.direction == RoadDirection::STRAIGHT) {
            double right_base_dis = (1 - p.lane_seq - 0.5) * p.lane_width;
            int    r_k            = p.lane_width / 0.5;
            for (int i = 1; i < r_k; ++i) {
              Pose block_p = p.getLateralPose(base_dis + i * 0.5);
              map.line_block_map[int(block_p.x)][int(block_p.y)] = 1;
            }
            double left_base_dis =
                (p.lane_num - p.lane_seq - 0.5) * p.lane_width;
            int l_k = p.lane_width / 0.5;
            for (int i = 1; i < l_k; ++i) {
              Pose block_p = p.getLateralPose(base_dis + i * 0.5);
              map.line_block_map[int(block_p.x)][int(block_p.y)] = 1;
            }
          }
        }
      }
    }
    // block dash line and boundary
    /*
    if (carInRoad()) {
      if (lane_line_block_type == LaneLineBlockType::ALL_BLOCK) {
        for (const auto& line : map.lane_line_list) {
          for (const auto& p : line) {
            if (p.type == LineType::SOLID)
              map.line_block_map[int(p.x)][int(p.y)] = 1;
          }
        }
        for (const auto& line : map.boundary_line) {
          for (const auto& p : line) {
            if (p.type == LineType::BOUNDARY)
              map.line_block_map[int(p.x)][int(p.y)] = 1;
          }
        }
      }
      if (lane_line_block_type == LaneLineBlockType::SEMI_BLOCK) {
        for (const auto& line : map.lane_line_list) {
          for (int i = int(line.size() * 2 / 3); i < line.size(); ++i) {
            auto& p = line[i];
            if (p.type == LineType::SOLID)
              map.line_block_map[int(p.x)][int(p.y)] = 1;
          }
        }
        for (const auto& line : map.boundary_line) {
          for (int i = int(line.size() * 2 / 3); i < line.size(); ++i) {
            auto& p = line[i];
            if (p.type == LineType::BOUNDARY)
              map.line_block_map[int(p.x)][int(p.y)] = 1;
          }
        }
      }
    } else {
      for (const auto& line : map.lane_line_list) {
        for (int i = int(line.size() * 2 / 3); i < line.size(); ++i) {
          auto& p = line[i];
          if (p.type == LineType::SOLID)
            map.line_block_map[int(p.x)][int(p.y)] = 1;
        }
      }
      for (const auto& line : map.boundary_line) {
        for (int i = int(line.size() * 2 / 3); i < line.size(); ++i) {
          auto& p = line[i];
          if (p.type == LineType::BOUNDARY || p.type == DASH)
            map.line_block_map[int(p.x)][int(p.y)] = 1;
        }
      }
    }
  */
  }
  // predict dynamic obs map
  predictDynamicObsInMap();
  getPlanningDisMap(history);
  getAccessibleMap();
}

Map& MapManager::getMap() { return map; }

vector<HDMapPoint> MapManager::getForwardRefPath() {
  ref_path_mutex.lock_shared();
  vector<HDMapPoint> res = map.forward_ref_path;
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

void MapManager::getPlanningDisMap(bool history) {
  const int    dx[]  = {0, 0, -1, 1, 1, 1, -1, -1};
  const int    dy[]  = {-1, 1, 0, 0, 1, -1, 1, -1};
  const double dis[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      map.planning_dis_map[r][c] = 1000000.0;
    }
  }
  queue<pair<int, int>> obj_que;

  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      if (!history &&
          (map.line_block_map[r][c] != 0 ||
           (map.lidar_map[r][c] != 0 && map.lidar_map[r][c] != 0x4) ||
           map.dynamic_obs_map[r][c] != 0)) {
        map.planning_dis_map[r][c] = 0;
        obj_que.push(make_pair(r, c));
      } else if (map.line_block_map[r][c] != 0 || map.lidar_map[r][c] & 0x1) {
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

void MapManager::getLaneLineList() {
  //清空之前的lane信息
  map.lane_line_list.clear();
  map.lane_center_list.clear();
  //判断车道数是否不变
  bool lane_num_change = false;
  int  lane_num        = map.forward_ref_path.front().lane_num;
  for (const auto& p : map.forward_ref_path) {
    if (p.lane_num != lane_num) {
      lane_num_change = true;
      break;
    }
  }
  map.lane_line_list.resize(lane_num + 1);
  map.lane_center_list.resize(lane_num);
  for (const auto& p : map.forward_ref_path) {
    //从右向左第i条地图车道线
    if (p.lane_num != lane_num || p.lane_num <= 0) break;
    for (int i = 0; i < lane_num + 1; ++i) {
      Pose     line_p = p.getLateralPose(p.lane_width * (i - p.lane_seq + 0.5));
      LineType line_type = LineType::DASH;
      if (p.mode == HDMapMode::INTERSECTION_SOLID) line_type = LineType::SOLID;
      if ((i == 0 && (p.block_type & BlockType::BlockRight)) ||
          (i == lane_num && (p.block_type & BlockType::BlockLeft)))
        line_type = LineType::BOUNDARY;
      if (line_p.in_map() && p.mode != HDMapMode::CHANGE)
        map.lane_line_list[i].emplace_back(line_p.x, line_p.y, line_type);
    }
    if (!lane_num_change) {
      //从右向左第i条地图车道中心线
      for (int i = 0; i < lane_num; ++i) {
        Pose line_p = p.getLateralPose(p.lane_width * (i - p.lane_seq + 1));
        if (line_p.in_map() && p.mode != HDMapMode::CHANGE)
          map.lane_center_list[i].emplace_back(line_p.x, line_p.y);
      }
    }
  }
  //若车道数改变了,获取远处的中心线,供选择目标点
  if (lane_num_change) {
    lane_num = map.forward_ref_path.back().lane_num;
    map.lane_center_list.resize(lane_num);
    for (int i = map.forward_ref_path.size() - 1; i >= 0; --i) {
      const HDMapPoint& p = map.forward_ref_path[i];
      if (p.lane_num != lane_num) break;
      //从右向左第j条地图车道中心线
      for (int j = 0; j < lane_num; ++j) {
        Pose line_p = p.getLateralPose(p.lane_width * (j - p.lane_seq + 1));
        if (line_p.in_map() && p.mode != HDMapMode::CHANGE)
          map.lane_center_list[j].emplace_back(line_p.x, line_p.y);
      }
    }
    for (auto& line : map.lane_center_list) reverse(line.begin(), line.end());
  }
}

vector<Pose> MapManager::getLaneTargets() {
  vector<Pose> targets;
  for (const auto& line : map.lane_center_list) {
    /*
    int    targets_id = -1;
    double s          = 0;
    if (line.size() == 0) continue;
    for (int i = 1; i < line.size(); ++i) {
      const Point2d& p = line[i];
      s += point2PointDis(line[i], line[i - 1]);
      if (point2PointDis(p, map.nav_info.car_pose) < 5 / GRID_RESOLUTION)
        continue;
      if (map.accessible_map[int(p.x)][int(p.y)]) targets_id = i;
      if (!p.in_map()) break;
      if (s > 70 / GRID_RESOLUTION) break;
    }
    const Point2d& pp  = line[targets_id];
    const Point2d& p   = line[targets_id - 1];
    Point2d        vec = pp - p;
    double         ang = vec.getRad();
    if (targets_id == -1)
      continue;
    else {
      targets.push_back(Pose(pp.x, pp.y, ang));
    }
  }
  */

    for (int i = line.size() - 1; i >= 0; --i) {
      const Point2d& p = line[i];
      if (point2PointDis(p, map.nav_info.car_pose) < 3 / GRID_RESOLUTION) break;
      if (map.accessible_map[int(p.x)][int(p.y)]) {
        double ang = PI;
        if (i - 1 >= 0) {
          const Point2d& pp  = line[i - 1];
          Point2d        vec = p - pp;
          ang                = vec.getRad();
        } else if (i + 1 < line.size()) {
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
    return this->getLaneTargets();
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

std::vector<Pose> MapManager::getTemporaryParkingTarget() {
  vector<Pose> targets;
  auto         current_tasks = this->getCurrentTasks();
  if (current_tasks.empty()) {
    Task parking_task = this->getParkingTask();
    if (parking_task.task_points.empty()) return targets;
    current_tasks.push_back(parking_task);
  }
  for (auto spot : current_tasks.back().task_points) {
    Pose p;
    p.utm_position = spot;
    p.updateLocalCoordinate(map.nav_info.car_pose);
    if (p.in_map() && map.accessible_map[int(p.x)][int(p.y)]) {
      targets.push_back(p);
      return targets;
    }
  }
  return targets;
}

vector<Pose> MapManager::getTaskTarget() {
  vector<Pose> targets;
  for (auto task_point : map.current_task_points.back().task_points) {
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

void MapManager::getBoundaryLine() {
  map.boundary_line.clear();
  bool lane_num_change     = false;
  int  lane_line_change_id = -1;
  int  mode_change_id      = -1;
  for (int i = 0; i < map.forward_ref_path.size(); ++i) {
    if (!lane_num_change && map.forward_ref_path[i].lane_num !=
                                map.forward_ref_path.front().lane_num) {
      lane_num_change     = true;
      lane_line_change_id = i;
    }
    if (map.forward_ref_path.front().mode != HDMapMode::CHANGE &&
        mode_change_id < 0 &&
        map.forward_ref_path[i].mode == HDMapMode::CHANGE) {
      mode_change_id = i;
    }
    if (lane_num_change && mode_change_id > 0) break;
  }
  vector<LinePoint> right_boundary;
  vector<LinePoint> left_boundary;
  for (const auto& p : map.ref_path) {
    if (p.s >= 0) break;
    if (p.mode == HDMapMode::CHANGE) continue;
    Pose right_line_p = p.getLateralPose(p.lane_width * (-p.lane_seq + 0.5));
    LineType right_line_type = LineType::DASH;
    if (p.block_type & BlockType::BlockRight)
      right_line_type = LineType::BOUNDARY;
    if (right_line_p.in_map())
      right_boundary.emplace_back(right_line_p.x, right_line_p.y,
                                  right_line_type);
    Pose left_line_p =
        p.getLateralPose(p.lane_width * (p.lane_num - p.lane_seq + 0.5));
    LineType left_line_type = LineType::DASH;
    if (p.block_type & BlockType::BlockLeft)
      left_line_type = LineType::BOUNDARY;
    if (left_line_p.in_map())
      left_boundary.emplace_back(left_line_p.x, left_line_p.y, left_line_type);
  }
  for (const auto& p : map.lane_line_list.front()) right_boundary.push_back(p);
  for (const auto& p : map.lane_line_list.back()) left_boundary.push_back(p);
  if (lane_num_change) {
    for (int j = lane_line_change_id; j < map.forward_ref_path.size(); ++j) {
      HDMapPoint p = map.forward_ref_path[j];
      if (p.mode == HDMapMode::CHANGE) continue;
      Pose right_line_p = p.getLateralPose(p.lane_width * (-p.lane_seq + 0.5));
      if (right_line_p.in_map())
        right_boundary.emplace_back(right_line_p.x, right_line_p.y,
                                    LineType::BOUNDARY);
      Pose left_line_p =
          p.getLateralPose(p.lane_width * (p.lane_num - p.lane_seq + 0.5));
      if (left_line_p.in_map())
        left_boundary.emplace_back(left_line_p.x, left_line_p.y,
                                   LineType::BOUNDARY);
    }
  } else if (mode_change_id > 0) {
    for (int j = mode_change_id; j < map.forward_ref_path.size(); ++j) {
      HDMapPoint p = map.forward_ref_path[j];
      if (p.mode == HDMapMode::CHANGE) continue;
      Pose right_line_p = p.getLateralPose(p.lane_width * (-p.lane_seq + 0.5));
      if (right_line_p.in_map())
        right_boundary.emplace_back(right_line_p.x, right_line_p.y,
                                    LineType::BOUNDARY);
      Pose left_line_p =
          p.getLateralPose(p.lane_width * (p.lane_num - p.lane_seq + 0.5));
      if (left_line_p.in_map())
        left_boundary.emplace_back(left_line_p.x, left_line_p.y,
                                   LineType::BOUNDARY);
    }
  }
  map.boundary_line.push_back(right_boundary);
  map.boundary_line.push_back(left_boundary);
  for (auto& line : map.boundary_line) {
    lineInterpolation(line);
    for (int n = 1; n < line.size(); ++n) {
      if (line[n].type == LineType::UNKNOWN_LINE)
        line[n].type = line[n - 1].type;
    }
    if (line.size() > 2) {
      const auto& p_end        = line.back();
      const auto& p_before_end = line[line.size() - 2];
      const auto  direction    = (p_end - p_before_end).getDirection();
      Point2d     extra_p      = p_end + direction * 2;
      while (extra_p.in_map()) {
        line.emplace_back(extra_p.x, extra_p.y, p_end.type);
        extra_p = extra_p + direction * 2;
      }
    }
  }
}

void MapManager::visualization() {
  MessageManager*   msgm = MessageManager::getInstance();
  visVISUALIZATION& vis  = msgm->visualization;
  msgm->setTextInfo();
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
  // best path
  vis.best_path.clear();
  vis.best_path_size = map.best_path.path.size();
  for (const auto& p : map.best_path.path) {
    visPoint vp;
    vp.x = p.x;
    vp.y = p.y;
    vis.best_path.push_back(vp);
  }
  // lanes and boundary
  vis.lanes.clear();
  vis.lanes_size = map.lane_line_list.size() + map.boundary_line.size();
  for (const auto& lane_line : map.lane_line_list) {
    visLaneLine vll;
    vll.lane_line_points_size = lane_line.size();
    for (const auto& p : lane_line) {
      visPoint vp;
      vp.x = p.x;
      vp.y = p.y;
      vll.lane_line_points.push_back(vp);
    }
    vis.lanes.push_back(vll);
  }
  for (const auto& boundary_line : map.boundary_line) {
    visLaneLine vll;
    vll.lane_line_points_size = boundary_line.size();
    for (const auto& p : boundary_line) {
      visPoint vp;
      vp.x = p.x;
      vp.y = p.y;
      vll.lane_line_points.push_back(vp);
    }
    vis.lanes.push_back(vll);
  }
  // speed planner
  msgm->publishVisualization();
}

vector<Pose> MapManager::getStartMaintainedPath() {
  std::vector<Pose> path = getMaintainedPath(map.nav_info);
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
  const double maintain_s   = 2 * map.nav_info.current_speed;
  int          maintain_idx = 0;
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
      if (p.backward)
        res.push_back(p);
      else
        break;
    }
  } else {
    for (auto p : path) {
      if (p.s < 0) continue;
      if (!p.backward)
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
      result_path.back().s);

  // anti-conversion
  for (auto& point : map.speed_maintained_path.path) {
    if (point.backward) {
      point.ang = NormalizeAngle(point.ang - PI);
      point.v   = -point.v;
      point.a   = -point.a;
    }
  }
}

void MapManager::predictDynamicObsInMap() {
  memset(map.dynamic_obs_map, 0, sizeof(map.dynamic_obs_map));
  const auto add_obs_between = [&](Point2d start, Point2d end) {
    Point2d direction = (end - start).getDirection();
    for (int i = 0; i < (end - start).len(); ++i) {
      Point2d now = start + direction * i;
      if (now.in_map()) map.dynamic_obs_map[(int)now.x][(int)now.y] = 1;
    }
  };
  for (const auto& obstacle : map.dynamic_obj_list.dynamic_obj_list) {
    if (obstacle.path.size() > 1) {
      Point2d start = obstacle.path[0];
      Point2d end   = obstacle.path[1];
      double  v     = (end - start).len();
      // if the obstacle's velocity lower than 2m/s, then it should be
      // regarded static obstacle
      if (v / GRID_RESOLUTION < 2) {
        for (int i = 0; i < obstacle.corners.size(); ++i) {
          const auto& start = obstacle.corners[i];
          const auto& end =
              obstacle.corners[(i + 1) % (int)obstacle.corners.size()];
          add_obs_between(start, end);
        }
      }
    }
  }
  return;
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
MapManager* MapManager::instance = new MapManager;
}  // namespace TiEV