#ifndef TiEV_CLASS_H
#define TiEV_CLASS_H

#include "config.h"
#include "const.h"
#include "pose.h"
#include <vector>
namespace TiEV {
class NavInfo {
public:
    bool   detected = false;
    Pose   car_pose;
    double current_speed;
    bool   reliable = false;
};

class SlamInfo {
public:
    bool   detected = false;
    double x;
    double y;
    double heading;
    bool   reliable = false;
};

class LidarMap {
public:
    bool detected = false;
    // map[i][j] represents a point at ith row and jth column
    // map is the lidar message detected when car is at current_position
    unsigned char map[MAX_ROW][MAX_COL];
};

class DynamicObj {
public:
    int             id      = -2;
    ObjectType      type    = UNKNOWN;
    double          width   = -1;
    double          length  = -1;
    double          heading = -1;
    vector<Pose>    path;
    vector<Point2d> corners;
};

class DynamicObjList {
public:
    bool               detected = false;
    vector<DynamicObj> dynamic_obj_list;
};

class WarningObj {
public:
    int    id;
    int    type;
    double x;
    double y;
};

class WarningObjList {
public:
    bool               detected = false;
    vector<WarningObj> warning_obj_list;
};
class TrafficLight {

public:
    bool detected = false;
    bool left     = true;
    bool straight = true;
    bool right    = true;
};

class Pedestrian {
public:
    bool            detected = false;
    vector<Point2d> positions;
};

class ParkingLot {
public:
    Point2d left_back;
    Point2d right_back;
    Point2d left_front;
    Point2d right_front;
};

class ParkingLotList {
public:
    bool               detected = false;
    vector<ParkingLot> parking_lot_list;
};

class LaneLine {
public:
    LineType          type;
    double            distance;
    vector<LinePoint> points;
};

class Lane {
public:
    int      type;
    double   width;
    Point2d  stop_point;
    LaneLine left_line;
    LaneLine right_line;
};

class LaneList {
public:
    bool         detected = false;
    int          current_id;
    vector<Lane> lane_list;
};

class RainSignal {
public:
    char signal;
};

class SecurityInfo {
public:
    SecurityInfo() {
        throw "unimplemented class";
    }
};
}
#endif /* TIEV_CLASS */
