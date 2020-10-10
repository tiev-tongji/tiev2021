#ifndef _ROAD_MANAGER_H_
#define _ROAD_MANAGER_H_

#include <iostream>
#include <pqxx/pqxx>
#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <map>
#include <mutex>
#include "nature/point.h"
#include "message_manager.h"
#include "config/config.h"
using namespace std;

namespace TiEV{


//Extra information of a point on refrence road
struct RoadInfo{
    int id, event, mode, speed_mode;
    int lane_num, lane_seq;
    double lane_width, curve;
    RoadDirection direction;
	BlockType block_type = BlockType::BlockAll;
};


static mutex road_manager_mtx;
class RoadManager{
public:
    static RoadManager* getInstance(){
		road_manager_mtx.lock();
		static RoadManager instance;
		road_manager_mtx.unlock();
		return &instance;
	}

    void readRoadMapFile(string path);

    bool initGlobalMap(string shp_path);

    bool findReferenceRoad(double start_point_lon, double start_point_lat, double end_point_lon, double end_point_lat, bool is_normal = true);

    /* Reference road returned has correct local coordinate and global coordinate data. */
    void getLocalReferencePath(const NavInfo& car_position, vector<Point>& reference_road, vector<RoadInfo>& extra_informations, bool need_opposite = false);

    /* Path to be maintained must have correct lat, lon, utmx, utmy and heading values. */
    void maintainPath(const NavInfo& car_position, const vector<Point>& path);

    void getMaintainedPath(const NavInfo& car_position, vector<Point>& maintained_path);

	void maintainParkingLotList(const NavInfo& nav_info, ParkingLotList const& parking_lot_list);

    void getMaintainedParkingLotList(const NavInfo& nav_info, ParkingLotList& parking_lot_list);

private:
	mutex maintained_path_mtx;
    MessageManager* message_manager;

    vector<Point> road_map;
    vector<RoadInfo> road_map_information;
    int rdmp_neareast_index = -1;

    vector<Point> maintained_path;
    bool maintained_path_published;

    ParkingLotList maintained_parking_lot_list;

    int getNearestPointIndex(const NavInfo& car_position, const vector<Point>& road);

    void filtPoints();

private:
	RoadManager(){
		message_manager = MessageManager::getInstance();
        //initGlobalMap("/home/autolab/tiev2019/env/postgres/map/v2/ref_line_all_v2.shp");
	};
	~RoadManager(){};
};

}

#endif
