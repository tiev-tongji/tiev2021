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
  MachineManager& mm  = MachineManager::getInstance();
  const Config&   cfg = Config::getInstance();
  // map managet initialization
  MapManager& mapm = MapManager::getInstance();
  if (cfg.enable_routing_by_file) mapm.readGlobalPathFile(cfg.roadmap_file);
  // FSM...
  // Context       context;
  // FSM::Instance machine{ context };
  while (true) {
    MessageManager& msgm = MessageManager::getInstance();
    msgm.clearTextInfo();
    time_t start_t = getTimeStamp();
    mapm.update();
    mm.context.update();  //更新途灵事件信息
    mm.machine.update();
    time_t end_t     = getTimeStamp();
    int    time_cost = (end_t - start_t) / 1000;
    if (mm.machine.isActive<NormalDriving>())
      msgm.addTextInfo("FSM State", "NormalDriving");
    if (mm.machine.isActive<FreeDriving>())
      msgm.addTextInfo("FSM State", "FreeDriving");
    if (mm.machine.isActive<GlobalPlanning>())
      msgm.addTextInfo("FSM State", "GlobalPlanning");
    if (mm.machine.isActive<GlobalReplanning>())
      msgm.addTextInfo("FSM State", "GlobalReplanning");
    if (mm.machine.isActive<ParkingPlanning>())
      msgm.addTextInfo("FSM State", "ParkingPlanning");
    if (mm.machine.isActive<OvertakeDriving>())
      msgm.addTextInfo("FSM State", "OvertakeDriving");
    if (mm.machine.isActive<TemporaryParkingPlanning>())
      msgm.addTextInfo("FSM State", "TemporaryParkingPlanning");
    if (mm.machine.isActive<TemporaryStop>())
      msgm.addTextInfo("FSM State", "TemporaryStop");
    if (mm.machine.isActive<Tracking>())
      msgm.addTextInfo("FSM State", "Tracking");
    if (mm.machine.isActive<IntersectionDriving>())
      msgm.addTextInfo("FSM State", "IntersectionDriving");
    mapm.visualization();
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
    auto      maintained_path = decision_context.getMaintainedPath();
    double    max_speed       = decision_context.getSpeedLimitMPS();
    SpeedPath speed_path;

    structAIMPATH control_path;
    control_path.points.clear();
    if (maintained_path.empty()) {
      // send a control path to stop
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
    } else {
      // do speed plan
      vector<pair<double, double>> speed_limits;
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
        speed_limits.emplace_back(
            point.s, min(max_speed, max_velocity_for_curvature(average_k)));
      }
      maintained_path.front().v = decision_context.getCarSpeedMPS();
      speed_path                = SpeedOptimizer::RunSpeedOptimizer(
          dynamic_list, maintained_path, speed_limits, maintained_path.back().s,
          decision_context.getCarSpeedMPS());
      // anti-conversion
      for (auto& point : speed_path.path) {
        if (point.backward) {
          point.ang = point.ang - M_PI;
          point.v   = -point.v;
          point.a   = -point.a;
        }
      }
      // send control trojectory
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
        if (tp.v < 0.8 && tp.v > 0.0000001) tp.v = 0.8;
        control_path.points.push_back(tp);
      }
    }

    // for (const auto& p : control_path.points) {
    //   std::cout << "x, y, ang: " << p.x << " , " << p.y << " , " << p.theta
    //   << std::endl;
    // }

    MessageManager::getInstance().publishPath(control_path);

    // visual maintained path
    visVISUALIZATION& vis = MessageManager::getInstance().visualization;
    vis.maintained_path.clear();
    vis.maintained_path_size = maintained_path.size();
    for (const auto& p : maintained_path) {
      visPoint vp;
      vp.x = p.x;
      vp.y = p.y;
      vis.maintained_path.push_back(vp);
    }
    MessageManager::getInstance().setSpeedPath(speed_path);
  }
}

/**
 *  This thread is updating the global path in realtime
 */
void updateGlobalPathFromMapServer() {
  time_t          start_time = getTimeStamp();
  MessageManager& msg_m      = MessageManager::getInstance();
  MapManager&     map_m      = MapManager::getInstance();
  MachineManager& mm         = MachineManager::getInstance();
  Routing&        routing    = Routing::getInstance();
  NavInfo         nav_info;
  const time_t    time_limit    = 2 * 1e6;
  const auto      duration_time = [&]() { return getTimeStamp() - start_time; };
  while (!Config::getInstance().enable_routing_by_file) {
    msg_m.getNavInfo(nav_info);
    start_time = getTimeStamp();
    if (Config::getInstance().taxi_mode) {
      routing.updateInfoToServer();
    }
    if (map_m.getForwardRefPath().empty()) continue;
    vector<HDMapPoint> tmp_global_path;
    HDMapMode          road_mode = map_m.getCurrentMapMode();
    // mode
    if (!nav_info.detected || road_mode == HDMapMode::INTERSECTION ||
        road_mode == HDMapMode::INTERSECTION_SOLID ||
        road_mode == HDMapMode::IN_PARK)
      continue;
    // fsm state
    if (mm.machine.isActive<GlobalReplanning>() ||
        mm.machine.isActive<ParkingFSM>() ||
        mm.machine.isActive<TemporaryParkingFSM>() ||
        mm.machine.isActive<IntersectionFSM>())
      continue;

    // the path ref path point relateve with car
    const auto car_ref_point = map_m.getForwardRefPath().front();
    TaskPoint  start_task_point(
        car_ref_point.lon_lat_position.lon, car_ref_point.lon_lat_position.lat,
        car_ref_point.utm_position.utm_x, car_ref_point.utm_position.utm_y,
        car_ref_point.utm_position.heading);
    int cost =
        routing.requestUpdateReferenceRoad(start_task_point, &tmp_global_path);
    if (cost < 0) {
      if (duration_time() < time_limit) {
        usleep(time_limit - duration_time());
      }
      continue;
    }
    LOG(INFO) << "Replace global path: size=" << tmp_global_path.size();
    if (!tmp_global_path.empty()) {
      map_m.setGlobalPath(tmp_global_path);
    }
    if (duration_time() < time_limit) {
      usleep(time_limit - duration_time());
    }
  }
}
}  // namespace TiEV
