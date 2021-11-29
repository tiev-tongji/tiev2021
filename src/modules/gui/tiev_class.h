#ifndef TiEV_CLASS_H
#define TiEV_CLASS_H

#include "pose.h"
#include <vector>

class NavInfo {
public:
    bool   detected = false;
    Pose   car_pose;
    double current_speed;
    bool   reliable = false;
    double lon;
    double lat;
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

enum DynamicType { CAR = 0, BYCICLE = 1, PEDESTRIAN = 2, UNKNOWN = 127 };
class DynamicObj {
public:
    int                  id      = -2;
    DynamicType          type    = UNKNOWN;
    double               width   = -1;
    double               length  = -1;
    double               heading = -1;
    double               v       = 0;  // m/s
    std::vector<Pose>    path;
    std::vector<Point2d> corners;
};

class DynamicObjList {
public:
    bool                    detected = false;
    std::vector<DynamicObj> dynamic_obj_list;
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
    bool                    detected = false;
    std::vector<WarningObj> warning_obj_list;
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
    bool                 detected = false;
    std::vector<Point2d> positions;
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
    bool                    detected = false;
    std::vector<ParkingLot> parking_lot_list;
};

class TiEVLaneLine {
public:
    LineType                   type;
    double                     distance;
    std::vector<TiEVLinePoint> points;
};

class TiEVLane {
public:
    int          type;
    double       width;
    TiEVLaneLine left_line;
    TiEVLaneLine right_line;
};

class LaneList {
public:
    bool                  detected = false;
    int                   current_id;
    std::vector<TiEVLane> lane_list;
};

class RainSignal {
public:
    bool detected = false;
    char signal;
};

class SecurityInfo {
public:
    SecurityInfo() {
        throw "unimplemented class";
    }
};

#endif /* TIEV_CLASS */
