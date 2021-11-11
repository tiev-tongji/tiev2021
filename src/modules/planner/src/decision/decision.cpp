#include "decision.h"

#include <unistd.h>

#include <thread>

#include "Routing.h"
#include "collision_check.h"
#include "config.h"
#include "decision_context.h"
#include "map_manager.h"
#include "tiev_utils.h"
#include "tievlog.h"

namespace TiEV {

/**
 * 决策规划的线程
 */
void runTiEVFSM() {
  MachineManager* mm  = MachineManager::getInstance();
  Config const*   cfg = Config::getInstance();
  // map managet initialization
  MapManager* mapm = MapManager::getInstance();
  if (cfg->enable_routing_by_file) mapm->readGlobalPathFile(cfg->roadmap_file);
  // FSM...
  // Context       context;
  // FSM::Instance machine{ context };
  while (true) {
    MessageManager* msgm = MessageManager::getInstance();
    msgm->clearTextInfo();
    time_t start_t = getTimeStamp();
    mapm->update();
    mm->context.update();  //更新途灵事件信息
    mm->machine.update();
    time_t end_t     = getTimeStamp();
    int    time_cost = (end_t - start_t) / 1000;
    if (mm->machine.isActive<NormalDriving>())
      msgm->addTextInfo("FSM State", "NormalDriving");
    if (mm->machine.isActive<BackTracking>())
      msgm->addTextInfo("FSM State", "BackTracking");
    if (mm->machine.isActive<Exploration>())
      msgm->addTextInfo("FSM State", "Exploration");
    if (mm->machine.isActive<FreeDriving>())
      msgm->addTextInfo("FSM State", "FreeDriving");
    if (mm->machine.isActive<GlobalPlanning>())
      msgm->addTextInfo("FSM State", "GlobalPlanning");
    if (mm->machine.isActive<GlobalReplanning>())
      msgm->addTextInfo("FSM State", "GlobalReplanning");
    if (mm->machine.isActive<IntersectionFreeDriving>())
      msgm->addTextInfo("FSM State", "IntersectionFreeDriving");
    if (mm->machine.isActive<LaneFreeDriving>())
      msgm->addTextInfo("FSM State", "LaneFreeDriving");
    if (mm->machine.isActive<ParkingPlanning>())
      msgm->addTextInfo("FSM State", "ParkingPlanning");
    if (mm->machine.isActive<ReplaceParkingPath>())
      msgm->addTextInfo("FSM State", "ReplaceParkingPath");
    if (mm->machine.isActive<SafeDriving>())
      msgm->addTextInfo("FSM State", "SafeDriving");
    if (mm->machine.isActive<SeekParkingSpot>())
      msgm->addTextInfo("FSM State", "SeekParkingSpot");
    if (mm->machine.isActive<OvertakeDriving>())
      msgm->addTextInfo("FSM State", "OvertakeDriving");
    if (mm->machine.isActive<Stop>()) msgm->addTextInfo("FSM State", "Stop");
    if (mm->machine.isActive<TaskDecision>())
      msgm->addTextInfo("FSM State", "TaskDecision");
    if (mm->machine.isActive<TemporaryParkingPlanning>())
      msgm->addTextInfo("FSM State", "TemporaryParkingPlanning");
    if (mm->machine.isActive<TemporaryStop>())
      msgm->addTextInfo("FSM State", "TemporaryStop");
    if (mm->machine.isActive<Tracking>())
      msgm->addTextInfo("FSM State", "Tracking");
    if (mm->machine.isActive<UTurn>()) msgm->addTextInfo("FSM State", "UTurn");
    msgm->addTextInfo("Time cost", to_string(time_cost));
    if (mm->machine.isActive<NormalDriving>())
      msgm->addTextInfo("FSM State", "NormalDriving");
    if (mm->machine.isActive<BackTracking>())
      msgm->addTextInfo("FSM State", "BackTracking");
    if (mm->machine.isActive<Exploration>())
      msgm->addTextInfo("FSM State", "Exploration");
    if (mm->machine.isActive<FreeDriving>())
      msgm->addTextInfo("FSM State", "FreeDriving");
    if (mm->machine.isActive<GlobalPlanning>())
      msgm->addTextInfo("FSM State", "GlobalPlanning");
    if (mm->machine.isActive<GlobalReplanning>())
      msgm->addTextInfo("FSM State", "GlobalReplanning");
    if (mm->machine.isActive<IntersectionFreeDriving>())
      msgm->addTextInfo("FSM State", "IntersectionFreeDriving");
    if (mm->machine.isActive<LaneFreeDriving>())
      msgm->addTextInfo("FSM State", "LaneFreeDriving");
    if (mm->machine.isActive<ParkingPlanning>())
      msgm->addTextInfo("FSM State", "ParkingPlanning");
    if (mm->machine.isActive<ReplaceParkingPath>())
      msgm->addTextInfo("FSM State", "ReplaceParkingPath");
    if (mm->machine.isActive<SafeDriving>())
      msgm->addTextInfo("FSM State", "SafeDriving");
    if (mm->machine.isActive<IntersectionDriving>())
      msgm->addTextInfo("FSM State", "IntersectionDriving");
    if (mm->machine.isActive<SeekParkingSpot>())
      msgm->addTextInfo("FSM State", "SeekParkingSpot");
    if (mm->machine.isActive<OvertakeDriving>())
      msgm->addTextInfo("FSM State", "OvertakeDriving");
    if (mm->machine.isActive<Stop>()) msgm->addTextInfo("FSM State", "Stop");
    if (mm->machine.isActive<TaskDecision>())
      msgm->addTextInfo("FSM State", "TaskDecision");
    if (mm->machine.isActive<TemporaryParkingPlanning>())
      msgm->addTextInfo("FSM State", "TemporaryParkingPlanning");
    if (mm->machine.isActive<TemporaryStop>())
      msgm->addTextInfo("FSM State", "TemporaryStop");
    if (mm->machine.isActive<Tracking>())
      msgm->addTextInfo("FSM State", "Tracking");
    if (mm->machine.isActive<UTurn>()) msgm->addTextInfo("FSM State", "UTurn");
    mapm->visualization();
  }
}
/* Before send the path to the trajectory controller, we must do the speed
 * planner for the path with some decison results:
 * 1.retrive the path, find if there is collion on the path
 * 2.add the pedestrian decision result
 * 3.add the traffic light decision result
 * the above decision results are transformed to virtual dynamic obj
 */
void sendPath() {
  // get the decison context
  const auto& decision_context = DecisionContext::getInstance();
  while (true) {
    const auto static_obstacle_virtual_dymanic =
        decision_context.getStaticObsDecision();
    const auto pedestrian_virtual_dymanic =
        decision_context.getPedestrianDecision();
    const auto traffic_light_virtual_dymanic =
        decision_context.getTrafficLightDecision();
    // add the above virtual dynamic objs to dynamic_list
    auto dynamic_list = decision_context.getDynamicList();
    dynamic_list.insert(dynamic_list.end(),
                        static_obstacle_virtual_dymanic.begin(),
                        static_obstacle_virtual_dymanic.end());
    dynamic_list.insert(dynamic_list.end(), pedestrian_virtual_dymanic.begin(),
                        pedestrian_virtual_dymanic.end());
    dynamic_list.insert(dynamic_list.end(),
                        traffic_light_virtual_dymanic.begin(),
                        traffic_light_virtual_dymanic.end());
    // do the speed plan for the maintained path
    const auto maintained_path = decision_context.getMaintainedPath();
    if (maintained_path.empty()) {
      // send a control path to stop
      // TODO
    } else {
      // do speed plan
      vector<pair<double, double>> speed_limits;
    }
  }
  //---old---
  MachineManager* mm   = MachineManager::getInstance();
  MapManager*     mapm = MapManager::getInstance();
  MessageManager* msgm = MessageManager::getInstance();
  structAIMPATH   control_path;
  NavInfo         nav_info;
  LidarMap        lidar;
  DynamicObjList  dynamic;
  RainSignal      rain_signal;
  TrafficLight    traffic_light;
  unsigned char   lidar_map[MAX_ROW][MAX_COL];
  double          lidar_dis_map[MAX_ROW][MAX_COL];
  const int       dx[]  = {0, 0, -1, 1, 1, 1, -1, -1};
  const int       dy[]  = {-1, 1, 0, 0, 1, -1, 1, -1};
  const double    dis[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};
  time_t          time_limit, last_update_time, duration_time;
  last_update_time = getTimeStamp();
  time_limit       = 1e4;  // 100Hz
  while (true) {
    duration_time    = getTimeStamp() - last_update_time;
    last_update_time = getTimeStamp();
    if (duration_time < time_limit) {
      usleep(time_limit - duration_time);
    }
    msgm->getMap(lidar);
    msgm->getNavInfo(nav_info);
    msgm->getDynamicObjList(dynamic);
    msgm->getRainSignal(rain_signal);
    msgm->getTrafficLight(traffic_light);
    // for lidar map and lidar dis map
    for (int r = 0; r < MAX_ROW; ++r) {
      for (int c = 0; c < MAX_COL; ++c) {
        lidar_dis_map[r][c] = 1000000.0;
      }
    }
    queue<pair<int, int>> obj_que;
    memset(lidar_map, 0, sizeof(lidar_map));
    if (lidar.detected) {
      for (int r = 0; r < MAX_ROW; ++r) {
        for (int c = 0; c < MAX_COL; ++c) {
          if (rain_signal.signal) {
            lidar_map[r][c] = (unsigned char)lidar.map[r][c] & 01;
            if (lidar_map[r][c] != 0) {
              lidar_dis_map[r][c] = 0;
              obj_que.push(make_pair(r, c));
            }
            continue;
          }
          if (lidar.map[r][c] & 04) continue;
          lidar_map[r][c] = lidar.map[r][c];
          if (lidar_map[r][c] != 0) {
            lidar_dis_map[r][c] = 0;
            obj_que.push(make_pair(r, c));
          }
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
    // get maintained path
    vector<Pose> maintained_path = mapm->getMaintainedPath(nav_info);

    for (auto& p : maintained_path) {
      p.v = 0;
      p.t = inf;
    }

    // run speed planner
    double        max_speed      = mapm->getCurrentMapSpeed();
    HDMapMode     road_mode      = mapm->getCurrentMapMode();
    RoadDirection road_direction = mapm->getCurrentRoadDirection();

    if (road_mode == HDMapMode::INTERSECTION_SOLID ||
        road_mode == HDMapMode::INTERSECTION ||
        road_mode == HDMapMode::PARKING) {
      max_speed = min(max_speed, mapm->getSpeedBySpeedMode(HDMapSpeed::LOW));
    }
    if (mm->machine.isActive<TemporaryParkingFSM>())
      max_speed =
          min(max_speed, mapm->getSpeedBySpeedMode(HDMapSpeed::VERY_LOW));
    if (mm->machine.isActive<TemporaryStop>() || mm->machine.isActive<Stop>())
      max_speed = 0;
    bool add_collision_dynamic = false;
    for (const auto& p : maintained_path) {
      if (!add_collision_dynamic && collision(p, lidar_dis_map)) {
        add_collision_dynamic = true;
        DynamicObj dummy_obj;
        dummy_obj.width  = 1.5;
        dummy_obj.length = 3;
        dummy_obj.path.emplace_back(p.x, p.y, p.ang, 0, 0, 0);
        dynamic.dynamic_obj_list.push_back(dummy_obj);
      }
    }
    // conversion
    vector<pair<double, double>> speed_limits;
    // int                          count = 0;
    for (int i = 0; i < maintained_path.size(); ++i) {
      auto& point = maintained_path[i];
      // get average k for near 5 points to smooth the k
      double average_k = 0.0;
      int    begin_idx = std::max(i - 10, 0);
      int    end_idx   = std::min(i + 10, int(maintained_path.size()) - 1);
      for (int j = begin_idx; j < end_idx; ++j) {
        average_k += maintained_path[j].k / (end_idx - begin_idx);
      }
      if (point.backward) {
        max_speed = min(2.0, max_speed);
        point.ang = M_PI + point.ang;
      }
      // LOG(INFO) << "hhhhhhhh"
      //           << "idx=" << count++ << " s=" << point.s << " \tk=" <<
      //           point.k
      //           << " \tave_k" << average_k
      //           << " \tv_by_k=" << max_velocity_for_curvature(average_k);

      speed_limits.emplace_back(
          point.s, min(max_speed, max_velocity_for_curvature(average_k)));
    }
    if (!maintained_path.empty())
      maintained_path.front().v = fabs(nav_info.current_speed);
    SpeedPath speed_path;
    // for(const auto &p:speed_limits)
    // cout << "speed limit:" << p.first << " "<<  p.second << endl;
    // if(!maintained_path.empty()) cout << "pid maintained path size:" <<
    // maintained_path.size() << " s=" << maintained_path.back().s << endl;
    if (!maintained_path.empty())
      speed_path = SpeedOptimizer::RunSpeedOptimizer(
          dynamic.dynamic_obj_list, maintained_path, speed_limits,
          maintained_path.back().s, nav_info.current_speed);
    // anti-conversion
    for (auto& point : speed_path.path) {
      if (point.backward) {
        point.ang = point.ang - M_PI;
        point.v   = -point.v;
        point.a   = -point.a;
      }
    }
    // send control trojectory
    control_path.points.clear();
    control_path.num_points = speed_path.path.size();
    for (const auto& p : speed_path.path) {
      TrajectoryPoint tp;
      tp.x     = (CAR_CEN_ROW - p.x) * GRID_RESOLUTION;
      tp.y     = (CAR_CEN_COL - p.y) * GRID_RESOLUTION;
      tp.theta = p.ang - PI;
      while (tp.theta > PI) tp.theta -= 2 * PI;
      while (tp.theta <= -PI) tp.theta += 2 * PI;
      tp.a = p.a;
      tp.k = p.k;
      tp.t = p.t;
      tp.v = p.v;
      if (road_mode == HDMapMode::IN_PARK)
        tp.v = min(tp.v, mapm->getCurrentMapSpeed());
      if (tp.v < 0.8 && tp.v > 0.0000001) tp.v = 0.8;
      control_path.points.push_back(tp);
    }
    if (control_path.points.empty()) {
      control_path.num_points = 10;
      for (int i = 0; i < 10; ++i) {
        TrajectoryPoint tp;
        tp.x     = 0.1 * i;
        tp.y     = 0.0;
        tp.theta = 0;
        tp.a     = 0;
        tp.k     = 0;
        tp.t     = 0.1 * i;
        tp.v     = 0;
        control_path.points.push_back(tp);
      }
    }
    // for(int i = 0; i < speed_path.path.size(); ++i) {
    //     Pose p = speed_path.path[i];
    //     cout << "pid speed path " << i << " " << p << endl;
    // }
    // for(int i = 0; i < control_path.points.size(); ++i) {
    //     TrajectoryPoint p = control_path.points[i];
    //     cout << "pid path " << i << ":{x=" << p.x << " y=" << p.y << "
    //     theta=" << p.theta << " a=" << p.a << " v=" << p.v << endl;
    // }

    msgm->publishPath(control_path);

    // visual maintained path
    visVISUALIZATION& vis = msgm->visualization;
    vis.maintained_path.clear();
    vis.maintained_path_size = maintained_path.size();
    for (const auto& p : maintained_path) {
      visPoint vp;
      vp.x = p.x;
      vp.y = p.y;
      vis.maintained_path.push_back(vp);
    }
    msgm->setSpeedPath(speed_path);
  }
}

void requestGlobalPathFromMapServer() {
  time_t          start_time = getTimeStamp();
  MessageManager* msg_m      = MessageManager::getInstance();
  MapManager*     map_m      = MapManager::getInstance();
  MachineManager* mm         = MachineManager::getInstance();
  Routing*        routing    = Routing::getInstance();
  NavInfo         nav_info;
  vector<Task>    task_list;
  const time_t    time_limit    = 10 * 1e6;
  const auto      duration_time = [&]() { return getTimeStamp() - start_time; };
  while (!Config::getInstance()->enable_routing_by_file) {
    msg_m->getNavInfo(nav_info);
    start_time = getTimeStamp();
    if (Config::getInstance()->taxi_mode) {
      routing->updateInfoToServer();
    }
    vector<HDMapPoint> tmp_global_path;
    HDMapMode          road_mode = map_m->getCurrentMapMode();
    // mode
    if (!nav_info.detected || road_mode == HDMapMode::INTERSECTION ||
        road_mode == HDMapMode::INTERSECTION_SOLID ||
        road_mode == HDMapMode::IN_PARK)
      continue;
    // fsm state
    if (mm->machine.isActive<UTurn>() ||
        mm->machine.isActive<GlobalReplanning>() ||
        mm->machine.isActive<ParkingFSM>() ||
        mm->machine.isActive<TemporaryParkingFSM>() ||
        mm->machine.isActive<IntersectionFSM>())
      continue;
    Task current_pos;
    current_pos.lon_lat_position.lon = nav_info.lon;
    current_pos.lon_lat_position.lat = nav_info.lat;
    task_list.clear();
    task_list.push_back(current_pos);
    vector<Task> current_tasks = map_m->getCurrentTasks();
    if (!current_tasks.empty())
      task_list.push_back(current_tasks.back());
    else
      task_list.push_back(map_m->getParkingTask());
    int cost = -1;
    if (task_list.size() > 1)
      cost = routing->findReferenceRoad(tmp_global_path, task_list, false);
    LOG(INFO) << "Cost of global path: " << cost;
    if (cost == -1) continue;
    LOG(INFO) << "global path size:" << tmp_global_path.size();
    if (!tmp_global_path.empty()) {  // TODO: When to replace?
      map_m->setGlobalPath(tmp_global_path);
      break;
    }
    if (duration_time() < time_limit) {
      usleep(time_limit - duration_time());
    }
  }
}
}  // namespace TiEV
