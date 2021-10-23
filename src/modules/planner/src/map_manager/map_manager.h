#ifndef _MAP_MANAGER_H
#define _MAP_MANAGER_H

#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include "Routing.h"
#include "const.h"
#include "message_manager.h"
#include "pose.h"

namespace TiEV {
struct Map {
  NavInfo        nav_info;
  SlamInfo       slam_info;
  TrafficLight   traffic_light;
  DynamicObjList dynamic_obj_list;
  WarningObjList warning_obj_list;
  ParkingLotList parking_lot_list;
  LaneList       lane_list;
  LidarMap       lidar;
  RainSignal     rain_signal;

  unsigned char lidar_map[MAX_ROW][MAX_COL];         //激光雷达地图
  unsigned char static_lidar_map[MAX_ROW][MAX_COL];  //激光雷达地图
  unsigned char lane_block_map[MAX_ROW][MAX_COL];    //车道线封闭地图
  unsigned char dynamic_obs_map[MAX_ROW][MAX_COL];   //
  double        lidar_dis_map[MAX_ROW][MAX_COL];  //激光障碍物距离地图
  double        planning_dis_map[MAX_ROW][MAX_COL];  //规划距离地图
  bool          accessible_map[MAX_ROW][MAX_COL];    //规划距离地图

  std::vector<HDMapPoint>             ref_path;
  std::vector<HDMapPoint>             forward_ref_path;
  std::vector<Pose>                   maintained_path;
  std::vector<std::vector<LinePoint>> v_line_list;  //视觉检测的车道线
  std::vector<std::vector<LinePoint>>
                                      lane_line_list;  //当前地图匹配好的车道线0.5m一个点
  std::vector<std::vector<LinePoint>> boundary_line;
  std::vector<std::vector<Point2d>>
                    lane_center_list;  //当前地图匹配好的车道线0.5m一个点
  std::vector<Pose> start_maintained_path;
  std::vector<Task>
                    current_tasks;  // The task points that have not been finished yet
  Task              parking_task;
  SpeedPath         best_path;
  SpeedPath         speed_maintained_path;
  std::vector<Pose> parking_spots;

  int car_lane_id;
};

/**
 * 向服务器请求全局路径
 * 决策规划规划地图的生成
 * 路径保持
 * 目标点保持
 */
class MapManager {
 public:
  void          update();  //更新map
  double        getSpeedBySpeedMode(int speed_mode);
  double        getCurrentMapSpeed();
  bool          requestGlobalPath(const NavInfo& nav_info);  //请求全局路
  void          readGlobalPathFile(const std::string& file_path);
  HDMapMode     getCurrentMapMode();
  RoadDirection getCurrentRoadDirection();
  HDMapPoint    getStopLine();
  vector<HDMapPoint> getForwardRefPath();
  HDMapSpeed         getCurrentSpeedMode();
  void updateRefPath(bool need_opposite = false);  //获取局部参考路
  void addPedestrian(DynamicObjList& dynamic_obj_list,
                     const vector<HDMapPoint>&
                         ref_path);  // 对道路内且相隔一定距离内的行人进行避让
  void blockStopLine();  // 封闭停止线，红灯时使用
  enum DynamicBlockType { NO_BLOCK, ALL_BLOCK };
  void updatePlanningMap(DynamicBlockType dynamic_block_type,
                         bool             history = false);
  Map& getMap();
  void visualization();
  void maintainParkingSpots();
  Task getParkingTask();
  void popCurrentTask();
  void pushCurrentTask(const Task&);  //添加新任务
  void clearTask();
  void setGlobalPath(const vector<HDMapPoint>& new_global_path);
  bool allowParking(const Pose&                    parking_spot,
                    const std::vector<HDMapPoint>& ref_path);

  vector<Task> getCurrentTasks();

  const std::vector<HDMapPoint> getLaneCenterDecision(const Map& decision_map);
  //获取目标点
  std::vector<Pose> getExplorationTargets();
  std::vector<Pose> getParkingSpotTarget();
  Pose              getTemporaryParkingTarget();
  std::vector<Pose> getTaskTarget();
  std::vector<Pose> getUTurnTargets();
  std::vector<Pose> maintained_uturn_target;
  // Pose getRefPathTarget(double s);
  //------
  std::vector<Pose> getMaintainedPath(const NavInfo& nav_info);
  void              getSpeedMaintainedPath(NavInfo& nav_info);
  std::vector<Pose> getStartMaintainedPath();
  void maintainPath(const NavInfo& nav_info, const vector<Pose>& path);
  void selectBestPath(const std::vector<SpeedPath>& paths);

 public:
  static MapManager* getInstance() { return instance; };

 protected:
  MapManager() {
    Config* config = Config::getInstance();
    //如果使用taxi模式，则在停车时从服务器获取任务
    if (!config->taxi_mode) {
      this->map.current_tasks = config->tasks;
    }
    //最终停车位置不变
    this->map.parking_task = config->parking_task;
  };

 private:
  Map    map;
  int    global_path_nearest_idx = -1;  // getRefPath中使用
  time_t global_path_update_time = -1;

  std::vector<HDMapPoint> global_path;
  std::vector<Pose>       maintained_path;
  static MapManager*      instance;
  std::shared_mutex       maintained_path_mutex;
  std::shared_mutex       ref_path_mutex;
  std::shared_mutex       global_path_mutex;
  std::shared_mutex       task_mutex;
  std::shared_mutex       parking_task_mutex;

 private:
  //---------execute when update--------
  void handleLidarMap();  // 根据LidarMap lidar获取unsigned char lidar_map
  void adjustRefPathByVLaneLine();  //通过视觉车道线矫正参考路
  void laneMatch();                 // 车道线匹配
  void dynamicDecision(const DynamicBlockType dynamic_block_type);
  void laneBlockDecision();
  void mapDecision(bool history = false);
  void laneLineInterpolation();
  void getAccessibleMap();
  //--------tool----------------
  int       getCarLaneId();  //获取车辆当前所在车道序号
  const int getGlobalPathNearestIndex(const int begin, const int end) const;
  bool vehicleIsOnRoad(Pose const& vehicle_pose);  // 判断车辆是否已回到车道
  void setGlobalPathDirection();  // 设置全局参考路中RoadDirection属性
  void filtPoints();
};
}  // namespace TiEV

#endif