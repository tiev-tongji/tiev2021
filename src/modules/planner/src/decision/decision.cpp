#include "decision.h"
#include "Routing.h"
#include "collision_check.h"
#include "config.h"
#include "map_manager.h"
#include "tiev_utils.h"
#include <thread>
#include <unistd.h>
namespace TiEV {

/**
 * 决策规划的线程
 */
void runTiEVFSM() {
    MachineManager* mm  = MachineManager::getInstance();
    Config const*   cfg = Config::getInstance();
    // map managet initialization
    MapManager* mapm = MapManager::getInstance();
    if(cfg->enable_routing_by_file) mapm->readGlobalPathFile(cfg->roadmap_file);
// Start a new thread for routing to update global path
// #define routing
#ifdef routing
    thread routing_thread = thread(&MapManager::runRouting, mapm, 10000000, false);
    routing_thread.detach();
#endif
    // FSM...
    // Context       context;
    // FSM::Instance machine{ context };
    while(true) {
        MessageManager* msgm = MessageManager::getInstance();
        msgm->clearTextInfo();
        time_t start_t = getTimeStamp();
        mapm->update();
        mm->context.update();  //更新途灵事件信息
        mm->machine.update();
        time_t end_t     = getTimeStamp();
        int    time_cost = (end_t - start_t) / 1000;
        msgm->addTextInfo("Time cost", to_string(time_cost));
        if(mm->machine.isActive<NormalDriving>()) msgm->addTextInfo("FSM State", "NormalDriving");
        if(mm->machine.isActive<BackTracking>()) msgm->addTextInfo("FSM State", "BackTracking");
        if(mm->machine.isActive<Exploration>()) msgm->addTextInfo("FSM State", "Exploration");
        if(mm->machine.isActive<FreeDriving>()) msgm->addTextInfo("FSM State", "FreeDriving");
        if(mm->machine.isActive<GlobalPlanning>()) msgm->addTextInfo("FSM State", "GlobalPlanning");
        if(mm->machine.isActive<GlobalReplanning>()) msgm->addTextInfo("FSM State", "GlobalReplanning");
        if(mm->machine.isActive<IntersectionFreeDriving>()) msgm->addTextInfo("FSM State", "IntersectionFreeDriving");
        if(mm->machine.isActive<LaneFreeDriving>()) msgm->addTextInfo("FSM State", "LaneFreeDriving");
        if(mm->machine.isActive<ParkingPlanning>()) msgm->addTextInfo("FSM State", "ParkingPlanning");
        if(mm->machine.isActive<ReplaceParkingPath>()) msgm->addTextInfo("FSM State", "ReplaceParkingPath");
        if(mm->machine.isActive<SafeDriving>()) msgm->addTextInfo("FSM State", "SafeDriving");
        if(mm->machine.isActive<SeekParkingSpot>()) msgm->addTextInfo("FSM State", "SeekParkingSpot");
        if(mm->machine.isActive<SemiLaneFreeDriving>()) msgm->addTextInfo("FSM State", "SemiLaneFreeDriving");
        if(mm->machine.isActive<Stop>()) msgm->addTextInfo("FSM State", "Stop");
        if(mm->machine.isActive<TaskDecision>()) msgm->addTextInfo("FSM State", "TaskDecision");
        if(mm->machine.isActive<TemporaryParkingPlanning>()) msgm->addTextInfo("FSM State", "TemporaryParkingPlanning");
        if(mm->machine.isActive<TemporaryStop>()) msgm->addTextInfo("FSM State", "TemporaryStop");
        if(mm->machine.isActive<Tracking>()) msgm->addTextInfo("FSM State", "Tracking");
        if(mm->machine.isActive<UTurn>()) msgm->addTextInfo("FSM State", "UTurn");
        mapm->visualization();
    }
}

void sendPath() {
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
    const int       dx[]  = { 0, 0, -1, 1, 1, 1, -1, -1 };
    const int       dy[]  = { -1, 1, 0, 0, 1, -1, 1, -1 };
    const double    dis[] = { 1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414 };
    while(true) {
        auto start_time = getTimeStamp();
        msgm->getMap(lidar);
        msgm->getNavInfo(nav_info);
        msgm->getDynamicObjList(dynamic);
        msgm->getRainSignal(rain_signal);
        msgm->getTrafficLight(traffic_light);
        // for lidar map and lidar dis map
        for(int r = 0; r < MAX_ROW; ++r) {
            for(int c = 0; c < MAX_COL; ++c) {
                lidar_dis_map[r][c] = 1000000.0;
            }
        }
        queue<pair<int, int>> obj_que;
        memset(lidar_map, 0, sizeof(lidar_map));
        if(lidar.detected) {
            for(int r = 0; r < MAX_ROW; ++r) {
                for(int c = 0; c < MAX_COL; ++c) {
                    if(rain_signal.signal) {
                        lidar_map[r][c] = (unsigned char)lidar.map[r][c] & 01;
                        if(lidar_map[r][c] != 0) {
                            lidar_dis_map[r][c] = 0;
                            obj_que.push(make_pair(r, c));
                        }
                        continue;
                    }
                    if(lidar.map[r][c] & 04) continue;
                    lidar_map[r][c] = lidar.map[r][c];
                    if(lidar_map[r][c] != 0) {
                        lidar_dis_map[r][c] = 0;
                        obj_que.push(make_pair(r, c));
                    }
                }
            }
        }
        while(!obj_que.empty()) {
            const int x = obj_que.front().first;
            const int y = obj_que.front().second;
            obj_que.pop();
            for(int i = 0; i < 8; ++i) {
                const int tx = x + dx[i], ty = y + dy[i];
                if(Point2d(tx, ty).in_map() && lidar_dis_map[tx][ty] > lidar_dis_map[x][y] + dis[i]) {
                    lidar_dis_map[tx][ty] = lidar_dis_map[x][y] + dis[i];
                    obj_que.push(make_pair(tx, ty));
                }
            }
        }
        // get maintained path
        vector<Pose> maintained_path = mapm->getMaintainedPath(nav_info);
        int          end_point       = maintained_path.size();
        for(int i = 1; i < maintained_path.size(); ++i)
            if(maintained_path[i].backward != maintained_path[i - 1].backward) {
                end_point = i;
                break;
            }

        vector<Pose> new_maintained_path;
        new_maintained_path.insert(new_maintained_path.begin(), maintained_path.begin() + end_point, maintained_path.end());
        for(auto& p : new_maintained_path) {
            p.v = 0;
            p.t = inf;
        }

        if(end_point != maintained_path.size()) {
            maintained_path.resize(end_point);
            maintained_path.back().v = 0;
        }

        // run speed planner
        double        max_speed      = mapm->getCurrentMapSpeed();
        HDMapMode     road_mode      = mapm->getCurrentMapMode();
        RoadDirection road_direction = mapm->getCurrentRoadDirection();

        if(road_mode == HDMapMode::INTERSECTION_SOLID || road_mode == HDMapMode::INTERSECTION || road_mode == HDMapMode::PARKING)
            max_speed = min(max_speed, mapm->getSpeedBySpeedMode(HDMapSpeed::LOW));
        if(mm->machine.isActive<SafeDriving>() || mm->machine.isActive<IntersectionFreeDriving>()) {
            mapm->addPedestrian(dynamic, mapm->getForwardRefPath());
        }
        // add stop line
        // cout << "Road direction:" << road_direction << endl;
        // cout << "Road mode:" << road_mode << endl;
        if(road_mode == HDMapMode::INTERSECTION_SOLID && traffic_light.detected) {
            if((road_direction == RoadDirection::LEFT && !traffic_light.left) || (road_direction == RoadDirection::STRAIGHT && !traffic_light.straight)
               || (road_direction == RoadDirection::RIGHT && !traffic_light.right)) {
                HDMapPoint stop_line = mapm->getStopLine();
                for(int i = 1; i <= stop_line.lane_num; ++i) {
                    double     dis             = (i - stop_line.lane_seq) * stop_line.lane_width;
                    Pose       other_stop_line = stop_line.getLateralPose(dis);
                    DynamicObj dummy_obj;
                    dummy_obj.width  = 1.5;
                    dummy_obj.length = 3;
                    dummy_obj.path.emplace_back(other_stop_line.x, other_stop_line.y, stop_line.ang, 0, 0, 0);
                    dynamic.dynamic_obj_list.push_back(dummy_obj);
                }
            }
        }
        if(mm->machine.isActive<TemporaryParkingFSM>()) max_speed                           = min(max_speed, mapm->getSpeedBySpeedMode(HDMapSpeed::VERY_LOW));
        if(mm->machine.isActive<TemporaryStop>() || mm->machine.isActive<Stop>()) max_speed = 0;
        bool add_collision_dynamic                                                          = false;
        for(const auto& p : maintained_path) {
            if(!add_collision_dynamic && collision(p, lidar_dis_map)) {
                add_collision_dynamic = true;
                DynamicObj dummy_obj;
                dummy_obj.width  = 1.5;
                dummy_obj.length = 3;
                dummy_obj.path.emplace_back(p.x, p.y, p.ang, 0, 0, 0);
                dynamic.dynamic_obj_list.push_back(dummy_obj);
            }
        }
        std::cout << "add collision dynamic:" << add_collision_dynamic << std::endl;
        for(const auto& dyo : dynamic.dynamic_obj_list) {
            std::cout << "dynamic_obj: id=" << dyo.id << " width=" << dyo.width << " length=" << dyo.length << " heading=" << dyo.heading << " path={";
            for(const auto& p : dyo.path)
                std::cout << "[" << p.x << " " << p.y << "]";
            std::cout << "}" << std::endl;
        }
        // conversion
        vector<pair<double, double>> speed_limits;
        for(auto& point : maintained_path) {
            if(point.backward) {
                max_speed = min(2.0, max_speed);
                point.ang = PI + point.ang;
            }
            speed_limits.emplace_back(point.s, min(sqrt(GRAVITY * MIU / (point.k + 0.0001)) * 0.6, max_speed));
        }
        if(!maintained_path.empty()) maintained_path.front().v = fabs(nav_info.current_speed);
        SpeedPath speed_path;
        // for(const auto &p:speed_limits)
        // cout << "speed limit:" << p.first << " "<<  p.second << endl;
        // if(!maintained_path.empty()) cout << "pid maintained path size:" <<
        // maintained_path.size() << " s=" << maintained_path.back().s << endl;
        if(!maintained_path.empty()) speed_path = SpeedOptimizer::RunSpeedOptimizer(dynamic.dynamic_obj_list, maintained_path, speed_limits, maintained_path.back().s);
        // anti-conversion
        for(auto& point : speed_path.path) {
            if(point.backward) {
                point.ang = point.ang - PI;
                point.v   = -point.v;
                point.a   = -point.a;
            }
        }
        // send control trojectory
        control_path.points.clear();
        control_path.num_points = speed_path.path.size();
        for(const auto& p : speed_path.path) {
            TrajectoryPoint tp;
            tp.x     = (CAR_CEN_ROW - p.x) * GRID_RESOLUTION;
            tp.y     = (CAR_CEN_COL - p.y) * GRID_RESOLUTION;
            tp.theta = p.ang - PI;
            while(tp.theta > PI)
                tp.theta -= 2 * PI;
            while(tp.theta <= -PI)
                tp.theta += 2 * PI;
            tp.a                                     = p.a;
            tp.k                                     = p.k;
            tp.t                                     = p.t;
            tp.v                                     = p.v;
            if(road_mode == HDMapMode::IN_PARK) tp.v = min(tp.v, mapm->getCurrentMapSpeed());
            if(tp.v < 0.5 && tp.v > 0.1) tp.v        = 0.5;
            control_path.points.push_back(tp);
        }
        if(control_path.points.empty()) {
            control_path.num_points = 10;
            for(int i = 0; i < 10; ++i) {
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
        for(int i = 0; i < control_path.points.size(); ++i) {
            TrajectoryPoint p = control_path.points[i];
            if(i > 10) break;
            cout << "pid path " << i << ":{x=" << p.x << " y=" << p.y << " theta=" << p.theta / PI * 180 << " a=" << p.a << " v=" << p.v << endl;
        }
        msgm->publishPath(control_path);
        // visual maintained path
        visVISUALIZATION& vis = msgm->visualization;
        vis.maintained_path.clear();
        vis.maintained_path_size = maintained_path.size();
        for(const auto& p : maintained_path) {
            visPoint vp;
            vp.x = p.x;
            vp.y = p.y;
            vis.maintained_path.push_back(vp);
        }
        msgm->setSpeedPath(speed_path);
        auto time_interval = getTimeStamp() - start_time;
        if(time_interval < 10 * 1000) usleep(10 * 1000 - time_interval);
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
    while(!Config::getInstance()->enable_routing_by_file) {
        msg_m->getNavInfo(nav_info);
        if(getTimeStamp() - start_time < 2e6) {
            usleep(2e6 + start_time - getTimeStamp());
        }
        start_time = getTimeStamp();
        if(Config::getInstance()->taxi_mode) {
            routing->updateInfoToServer();
        }
        vector<HDMapPoint> tmp_global_path;
        HDMapMode          road_mode = map_m->getCurrentMapMode();
        // mode
        if(!nav_info.detected || road_mode == HDMapMode::INTERSECTION || road_mode == HDMapMode::INTERSECTION_SOLID || road_mode == HDMapMode::IN_PARK) continue;
        // fsm state
        if(mm->machine.isActive<UTurn>() || mm->machine.isActive<GlobalReplanning>() || mm->machine.isActive<ParkingFSM>() || mm->machine.isActive<TemporaryParkingFSM>()
           || mm->machine.isActive<IntersectionFSM>())
            continue;
        Task current_pos;
        current_pos.lon_lat_position.lon = nav_info.lon;
        current_pos.lon_lat_position.lat = nav_info.lat;
        task_list.clear();
        task_list.push_back(current_pos);
        vector<Task> current_tasks = map_m->getCurrentTasks();
        if(!current_tasks.empty())
            task_list.push_back(current_tasks.back());
        else
            task_list.push_back(map_m->getParkingTask());
        int cost                      = -1;
        if(task_list.size() > 1) cost = routing->findReferenceRoad(tmp_global_path, task_list, false);
        cout << "Cost of global path: " << cost << endl;
        if(cost == -1) continue;
        cout << "global path size:" << tmp_global_path.size() << endl;
        if(!tmp_global_path.empty()) {  // TODO: When to replace?
            map_m->setGlobalPath(tmp_global_path);
        }
    }
}
}  // namespace TiEV
