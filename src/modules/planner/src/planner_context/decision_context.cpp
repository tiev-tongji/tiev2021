#include "decision_context.h"

#include "message_manager.h"

namespace TiEV {
const std::vector<DynamicObj>& DecisionContext::getPedestrianDecision() const {
  std::shared_lock<std::shared_mutex> lck(pedestrian_mutex);
  return _pedestrian_decision_result;
}

const std::vector<DynamicObj>& DecisionContext::getTrafficLightDecision()
    const {
  std::shared_lock<std::shared_mutex> lck(traffic_light_mutex);
  return _traffic_light_decision_result;
}

const std::vector<DynamicObj> DecisionContext::getStaticObsDecision() const {
  LidarMap lidar_map;
  MessageManager::getInstance().getMap(lidar_map);
  std::vector<DynamicObj> static_obs_decision_result;
  if (!lidar_map.detected) return static_obs_decision_result;

  double       lidar_dis_map[MAX_ROW][MAX_COL];
  const int    dx[]  = {0, 0, -1, 1, 1, 1, -1, -1};
  const int    dy[]  = {-1, 1, 0, 0, 1, -1, 1, -1};
  const double dis[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};
  // for lidar map and lidar dis map
  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      lidar_dis_map[r][c] = std::numeric_limits<double>::max();
    }
  }
  queue<pair<int, int>> obj_que;
  for (int r = 0; r < MAX_ROW; ++r) {
    for (int c = 0; c < MAX_COL; ++c) {
      if (lidar_map.map[r][c] & 04) lidar_map.map[r][c] = 0;
      if (lidar_map.map[r][c] != 0) {
        lidar_dis_map[r][c] = 0;
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
          lidar_dis_map[tx][ty] > lidar_dis_map[x][y] + dis[i]) {
        lidar_dis_map[tx][ty] = lidar_dis_map[x][y] + dis[i];
        obj_que.push(make_pair(tx, ty));
      }
    }
  }
  // static obs decision
  for (const auto& point : getMaintainedPath()) {
    if (collision(point, lidar_dis_map)) {
      DynamicObj obj;
      obj.type = CAR;
      obj.v    = 0;
      obj.path.emplace_back(point.x, point.y, point.ang, 0, 0, 0);
      obj.width  = 2;
      obj.length = 5;
      static_obs_decision_result.push_back(obj);
      break;
    }
  }
  return static_obs_decision_result;
}

const std::vector<DynamicObj> DecisionContext::getDynamicList() const {
  DynamicObjList dynamic_list;
  MessageManager::getInstance().getDynamicObjList(dynamic_list);
  return dynamic_list.dynamic_obj_list;
}

const std::vector<Pose> DecisionContext::getMaintainedPath() const {
  std::shared_lock<std::shared_mutex> lck(maintained_path_mutex);
  auto                                path = _maintained_path;
  lck.unlock();
  NavInfo nav_info;
  MessageManager::getInstance().getNavInfo(nav_info);
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
  return res;
}

const std::vector<Pose> DecisionContext::getMaintainedPath(
    const NavInfo& nav_info) const {
  std::shared_lock<std::shared_mutex> lck(maintained_path_mutex);
  auto                                path = _maintained_path;
  lck.unlock();
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
  return res;
}

const std::queue<PlannerInfo> DecisionContext::getPlannerHitory() const {
  std::shared_lock<std::shared_mutex> lck(planner_info_mutex);
  return _planner_history_buffer;
}

const double DecisionContext::getSpeedLimitMPS() const {
  std::shared_lock<std::shared_mutex> lck(speed_limit_mutex);
  return _speed_limit_mps;
}

const double DecisionContext::getSpeedLimitKPH() const {
  std::shared_lock<std::shared_mutex> lck(speed_limit_mutex);
  return _speed_limit_mps * 3.6;
}

const double DecisionContext::getCarSpeedMPS() const {
  NavInfo nav_info;
  MessageManager::getInstance().getNavInfo(nav_info);
  return nav_info.current_speed;
}

const PlanningWeights& DecisionContext::getPlanningWeights() const {
  return _weights;
}

void DecisionContext::setPedestrianDecision(
    const std::vector<DynamicObj>& pedestrian_decision_result) {
  std::unique_lock<std::shared_mutex> lck(pedestrian_mutex);
  _pedestrian_decision_result.clear();
  _pedestrian_decision_result = pedestrian_decision_result;
}

void DecisionContext::setTrafficLightDecision(
    const std::vector<DynamicObj>& traffic_light_decision_result) {
  std::unique_lock<std::shared_mutex> lck(traffic_light_mutex);
  _traffic_light_decision_result.clear();
  _traffic_light_decision_result = traffic_light_decision_result;
}

void DecisionContext::setMaintainedPath(const std::vector<Pose> path) {
  std::unique_lock<std::shared_mutex> lck(maintained_path_mutex);
  _maintained_path.clear();
  _maintained_path = path;
}

void DecisionContext::setPlannerInfo(const PlannerInfo& planner_info) {
  std::unique_lock<std::shared_mutex> lck(planner_info_mutex);
  while (_planner_history_buffer.size() > max_buffer_size) {
    _planner_history_buffer.pop();
  }
  _planner_history_buffer.push(planner_info);
}

void DecisionContext::setSpeedLimitMPS(const double speed_limit_mps) {
  std::unique_lock<std::shared_mutex> lck(speed_limit_mutex);
  _speed_limit_mps = speed_limit_mps;
}

void DecisionContext::setPlanningWeights(const PlanningWeights& weights) {
  _weights = weights;
}
}  // namespace TiEV
