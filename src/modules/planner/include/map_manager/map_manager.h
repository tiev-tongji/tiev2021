#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
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
    SpeedPath                           best_path;

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
    void update();                                    //更新map
    bool requestGlobalPath(const NavInfo& nav_info);  //请求全局路
    void readGlobalPathFile(const std::string& file_path);
    void updateRefPath(bool need_opposite = false);  //获取局部参考路
    void avoidPedestrian();                          // 对道路内且相隔一定距离内的行人进行避让
    enum LaneLineBlockType { NO_BLOCK, SEMI_BLOCK, ALL_BLOCK };
    void updatePlanningMap(LaneLineBlockType lane_line_block_type);
    Map& getMap();
    void visualization();
    //获取目标点
    std::vector<Pose> getLaneTargets();
    Pose              getBackTarget();
    Pose              getExplorationTarget();
    Pose              getParkingSpotTarget();
    vector<Pose>      getTaskTargets();
    //------
    std::vector<Pose> getMaintainedPath();
    std::vector<Pose> getStartMaintainedPath();
    void              maintainPath();
    void selectBestPath(const std::vector<SpeedPath>& paths);

public:
    static MapManager* getInstance() {
        return instance;
    };

protected:
    MapManager(){};

private:
    Map map;
    int global_path_nearest_idx = -1;  // getRefPath中使用

    std::vector<HDMapPoint> global_path;
    std::vector<Pose>       maintained_path;
    static MapManager*      instance;
    shared_mutex            maintained_path_mutex;

private:
    //---------execute when update--------
    void handleLidarMap();            // 根据LidarMap lidar获取unsigned char lidar_map
    void adjustRefPathByVLaneLine();  //通过视觉车道线矫正参考路
    void getLaneLineList();           //结果为map.lane_line_list和map.lane_center_list
    void laneMatch();                 // 车道线匹配
    void getBoundaryLine();
    void laneLineInterpolation();
    void getLidarDisMap();
    void getPlanningDisMap();
    void getAccessibleMap();
    //--------tool----------------
    int  getCarLaneId();                             //获取车辆当前所在车道序号
    bool vehicleIsOnRoad(Pose const& vehicle_pose);  // 判断车辆是否已回到车道
    int  getGlobalPathNearestIndex() const;
    void setGlobalPathDirection();  // 设置全局参考路中RoadDirection属性
    void filtPoints();
    void updateMaintainedPath();
};
}  // namespace TiEV

#endif