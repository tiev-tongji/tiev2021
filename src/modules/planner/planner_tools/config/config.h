#ifndef _TIEV_PLANNER_CONFIG_H_
#define _TIEV_PLANNER_CONFIG_H_

#include <map>
#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <cstring>
#include <mutex>
#include <vector>
#include "nature/point.h"
using namespace std;


namespace TiEV {

enum SpeedMode {
	BACK = 0,
	STOP = 1,
	VERYLOW = 2,
	LOW = 3,
	MID = 4,
	HIGH = 5,
	VERYHIGH = 6
};

enum ControlMode {
    PlanningWithMapMode,    //规划模式，使用地图RoadMode和SpeedMode
    TrakingWithMapMode, //循迹模式，使用地图RoadMode和SpeedMode
    PlanningWithDebugMode,  //规划模式，使用debugRoadMode和debugSpeedMode
    TrakingWithDebugMode    //循迹模式，使用debugRoadMode和debugSpeedMode
};

enum RoadMode {
    START_MODE = 0, //车辆启动
    UTURN_MODE = 1, //U字调头
    PEDESTRIAN_MODE = 2,  //人行道
    INTERSECTION = 3, //路口
    INTERSECT_NOLIGHT_MODE = 4, //汇入车流
    MAPFREE_MODE = 5, //无可靠地图
    OBSTACLFREE_MODE = 6, //无障碍物探测
    CLIMB_MODE = 7, //上下坡
    STOPLINE_MODE = 8,
    TOLLSTATION_MODE = 9,
    NORMAL_DRIVING_MODE = 10, //正常行驶，使用默认权值
    TRACKING_MODE = 11,
    EMERGENCY_MODE = 12,
    BACKWARD_MODE = 13,
    FOGGY_MODE = 14,
    SIDEPARKING_MODE = 15,
    STOP_MODE = 16,
    PARKING_MODE = 17   //正在停车
};

enum EventPoint {
	STOP_POINT = 1,
	TMP_STOP_POINT = 2,
	STOP_LINE = 3
};

enum RoadDirection{
	LEFT = 0x04,
	STRAIGHT = 0x02,
	RIGHT = 0x01
};

enum BlockType{
	BlockLeft = 2,
	BlockRight = 1,
	BlockAll = 0,
	BlockNone = 3
};

enum LineType{
	SOLID,
	DASHED
};

enum ObjectType{
	CAR = 0,
	BYCICLE = 1,
	PEDESTRIAN = 2,
	UNKNOWN = 127
};

struct Task{
    vector<Point> task_points;
};

static mutex config_mtx;

class Config {
public:
	const string TiEV_CONFIG_DIRECT = "/home/autolab/tiev2019/src/cfg/";
    //NOTICE: READ-ONLY
    string roadmap_file;
    //NOTICE: READ-ONLY
    ControlMode control_mode;
    //NOTICE: READ-ONLY
    RoadMode debug_event_mode;
    //NOTICE: READ-ONLY
    SpeedMode debug_speed_mode;

    //NOTICE: READ-ONLY
    int targets_num_limit;
    //NOTICE: READ-ONLY
    int plan_time_limit_ms;
    //NOTICE: READ-ONLY
    int car_away_limit_meter;

    //NOTICE: READ-ONLY
    string rs_distance_table_path;
    //NOTICE: READ-ONLY
    string dubins_distance_table_path;
    //NOTICE: READ-ONLY
    double a_star_extention_step_meter;
    //NOTICE: READ-ONLY
    double a_star_analytic_expansion_param_k;
    //NOTICE: READ-ONLY
    double a_star_analytic_expansion_param_t;
    //NOTICE: READ-ONLY
    int a_star_analytic_expansion_max_N;
    //NOTICE: READ-ONLY
    double a_star_curvature_changed_punishment;
    //NOTICE: READ-ONLY
	vector<Task> tasks;

    static Config* getInstance(){
        config_mtx.lock();
        static Config inner_instance;
        config_mtx.unlock();
        return &inner_instance;
    }

private:
    Config(){
		init();
	}

    ~Config(){}

	void init();

    void outputConfigures() const;
};

}

#endif
