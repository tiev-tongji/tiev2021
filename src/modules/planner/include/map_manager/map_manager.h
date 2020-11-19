#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include "Routing.h"
#include "const.h"
#include "message_manager.h"
#include "pose.h"
#include <string>
#include <vector>

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
    unsigned char line_block_map[MAX_ROW][MAX_COL];    //车道线封闭地图
    unsigned char dynamic_obs_map[MAX_ROW][MAX_COL];   //
    double        lidar_dis_map[MAX_ROW][MAX_COL];     //激光障碍物距离地图
    double        planning_dis_map[MAX_ROW][MAX_COL];  //规划距离地图
    bool          accessible_map[MAX_ROW][MAX_COL];    //规划距离地图

    std::vector<HDMapPoint>             ref_path;
    std::vector<HDMapPoint>             forward_ref_path;
    std::vector<Pose>                   maintained_path;
    std::vector<std::vector<LinePoint>> v_line_list;     //视觉检测的车道线
    std::vector<std::vector<LinePoint>> lane_line_list;  //当前地图匹配好的车道线0.5m一个点
    std::vector<std::vector<LinePoint>> boundary_line;
    std::vector<std::vector<Point2d>>   lane_center_list;  //当前地图匹配好的车道线0.5m一个点
    std::vector<Pose>                   start_maintained_path;
    std::vector<Task>                   current_task_points;  // The task points that have not been finished yet
    Task                                parking_task;
    SpeedPath                           best_path;
    SpeedPath                           speed_maintained_path;
    std::vector<Pose>                   parking_spots;

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
    void   update();  //更新map
    double getSpeedBySpeedMode(int speed_mode);
    double getCurrentMapSpeed();
    bool requestGlobalPath(const NavInfo& nav_info);  //请求全局路
    void readGlobalPathFile(const std::string& file_path);
    void runRouting(int interval, bool blocked);  // Update global path in a new thread
    HDMapMode          getCurrentMapMode();
    RoadDirection      getCurrentRoadDirection();
    HDMapPoint         getStopLine();
    vector<HDMapPoint> getForwardRefPath();
    void updateRefPath(bool need_opposite = false);                                            //获取局部参考路
    void addPedestrian(DynamicObjList& dynamic_obj_list, const vector<HDMapPoint>& ref_path);  // 对道路内且相隔一定距离内的行人进行避让
    void blockStopLine();                                                                      // 封闭停止线，红灯时使用
    enum LaneLineBlockType { NO_BLOCK, SEMI_BLOCK, ALL_BLOCK };
    void updatePlanningMap(LaneLineBlockType lane_line_block_type, bool history = false);
    Map&         getMap();
    void         visualization();
    void         maintainParkingSpots();
    vector<Task> getCurrentTasks();
    Task         getParkingTask();
    void         popCurrentTask();
    void         clearTask();
    bool         carInRoad();
    void setGlobalPath(const vector<HDMapPoint>& new_global_path);
    //获取目标点
    std::vector<Pose> getLaneTargets();
    Pose              getBackTarget();
    std::vector<Pose> getExplorationTargets();
    std::vector<Pose> getParkingSpotTarget();
    std::vector<Pose> getTemporaryParkingTarget();
    std::vector<Pose> getTaskTarget();
    std::vector<Pose> getUTurnTargets();
    // Pose getRefPathTarget(double s);
    //------
    std::vector<Pose> getMaintainedPath(NavInfo& nav_info);
    void getSpeedMaintainedPath(NavInfo& nav_info);
    void              predictDynamicObsInMap();
    std::vector<Pose> getStartMaintainedPath();
    void maintainPath(NavInfo& nav_info, vector<Pose>& path);
    void selectBestPath(const std::vector<SpeedPath>& paths);

public:
    static MapManager* getInstance() {
        return instance;
    };

protected:
    MapManager() {
        Config* config                = Config::getInstance();
        this->map.current_task_points = config->tasks;
        this->map.parking_task        = config->parking_task;
    };

private:
    Map    map;
    int    global_path_nearest_idx = -1;  // getRefPath中使用
    time_t global_path_update_time = -1;

    std::vector<HDMapPoint> global_path;
    std::vector<Pose>       maintained_path;
    static MapManager*      instance;
    shared_mutex            maintained_path_mutex;
    shared_mutex            ref_path_mutex;
    shared_mutex            global_path_mutex;
    shared_mutex            task_points_mutex;
    shared_mutex            parking_task_mutex;

private:
    //---------execute when update--------
    void handleLidarMap();            // 根据LidarMap lidar获取unsigned char lidar_map
    void adjustRefPathByVLaneLine();  //通过视觉车道线矫正参考路
    void getLaneLineList();           //结果为map.lane_line_list和map.lane_center_list
    void laneMatch();                 // 车道线匹配
    void getBoundaryLine();
    void laneLineInterpolation();
    void getPlanningDisMap(bool history = false);
    void getAccessibleMap();
    //--------tool----------------
    int  getCarLaneId();                             //获取车辆当前所在车道序号
    bool vehicleIsOnRoad(Pose const& vehicle_pose);  // 判断车辆是否已回到车道
    int getGlobalPathNearestIndex(int begin, int end) const;
    void setGlobalPathDirection();  // 设置全局参考路中RoadDirection属性
    void filtPoints();
    void updateMaintainedPath();
};
}  // namespace TiEV

#endif