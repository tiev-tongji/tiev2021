#include "road_manager.h"
#include <fstream>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include "smooth/smooth.h"
#include <unistd.h>
#include "utils/utils.h"
#include "interpolator/interpolator.h"
#include "circle_fit/CircleFitByTaubin.h"

using namespace std;

const double inf = 1e10;

namespace TiEV{

void RoadManager::readRoadMapFile(string path){
    const string error_header = "Read road-map file failed : ";

    road_map.clear();
    road_map_information.clear();
    fstream input(path, ios::in);
    if(!input.is_open()){
        cout << error_header << "file not exist." << endl;
        return;
    }

    string buffer, field;
    getline(input, buffer);
    stringstream buffer_stream(buffer);

    //check number of fields
    /*
    const int field_num = 12;
    int field_num_in_file = 0;
    */
	/*
    while(buffer_stream >> field){
        cout << "Add field: " << field << endl;
        ++field_num_in_file;
    }*/

    //if(field_num_in_file != field_num){
    //    cout << error_header << "content of input file is in illegal format." << endl;
    //    return;
    //}

    RoadInfo information;
    double lat, lon, ang;
    int block_type;
    while(input >> information.id >> lon >> lat >> ang >>
        information.curve >> information.mode >> information.speed_mode >>
        information.event >> block_type >> information.lane_num >> information.lane_seq >> information.lane_width){
            Point point = Point::fromLatLon(lat, lon);
            point.heading.setByRad(ang);
            road_map.push_back(point);
            information.block_type = (BlockType)block_type;
			information.lane_width += 0.5;
            road_map_information.push_back(information);
    }


	cout << "Road_map.size() " << road_map.size() << endl;
    for(int i = 0; i < road_map.size();){
        int inter_entry = road_map.size() - 1;
        for(int j = i + 1; j < road_map.size(); ++j)
            if(road_map_information[j].mode == RoadMode::INTERSECTION
                && road_map_information[j].event == EventPoint::STOP_LINE){
                //Enter intersection
                inter_entry = j;
                break;
            }
        int inter_exit = road_map.size() - 1;
        for(int j = inter_entry + 1; j < road_map.size(); ++j)
            if(road_map_information[j].event == EventPoint::STOP_LINE){
                //Exit intersection
                inter_exit = j;
                break;
            }
        double delta_ang = road_map[inter_exit].heading.getRad() -
            road_map[inter_entry].heading.getRad();
        utils::normalizeAngle(delta_ang);

        RoadDirection dir = RoadDirection::STRAIGHT;
        if(delta_ang > Angle::PI * 0.833333 || delta_ang < -Angle::PI * 0.833333)
            dir = RoadDirection::LEFT;
        else if(delta_ang > Angle::PI / 6.0) dir = RoadDirection::LEFT;
        else if(delta_ang < - Angle::PI / 6.0) dir = RoadDirection::RIGHT;
        else dir = RoadDirection::STRAIGHT;

        for(int j = i; j <= inter_exit; ++j)
            road_map_information[j].direction = dir;
		i = inter_exit + 1;
    }

    filtPoints();
	cout << "Road_map.size() after filtPoints " << road_map.size() << endl;
}

void RoadManager::filtPoints(){
    vector<Point> tmp_points;
    vector<RoadInfo> tmp_infos;
    tmp_points.reserve(road_map.size());
    tmp_infos.reserve(road_map.size());
    double priv_ang = 0, cur_ang = 0, next_ang = 0;
    int priv_lane = 0, cur_lane = 0, next_lane = 0;
    if(!road_map.empty()){
        tmp_points.push_back(road_map.front());
        tmp_infos.push_back(road_map_information.front());
    }

    for(int i = 1; i < road_map.size() - 1; ++i){
        if(road_map_information[i].event != EventPoint::STOP_LINE){
            priv_ang = road_map[i-1].heading.getRad();
            cur_ang = road_map[i].heading.getRad();
            next_ang = road_map[i+1].heading.getRad();
			double delta_ang = priv_ang - next_ang;
			utils::normalizeAngle(delta_ang);
            double std_ang = delta_ang / 2.0 + next_ang;
            utils::normalizeAngle(std_ang);
            utils::normalizeAngle(cur_ang);
            if(fabs(cur_ang - std_ang) > 0.6) continue;

            priv_lane = road_map_information[i-1].lane_num;
            cur_lane = road_map_information[i].lane_num;
            next_lane = road_map_information[i+1].lane_num;
            if(priv_lane == next_lane && next_lane != cur_lane) continue;
        }

        tmp_points.push_back(road_map[i]);
        tmp_infos.push_back(road_map_information[i]);
    }

    if(road_map.size() > 1){
        tmp_points.push_back(road_map.back());
        tmp_infos.push_back(road_map_information.back());
    }

    swap(road_map, tmp_points);
    swap(road_map_information, tmp_infos);
}

bool RoadManager::initGlobalMap(string shp_path)
{
    try
    {
        string database("changshu");
        string user("postgres");
        string password("postgres");
        string order;
        order = "shp2pgsql -s 4326 -d " + shp_path + " jiugongge_road |psql -h 127.0.0.1 -p 5432 -U " + user + " -d " + database;
        system(order.c_str());
        std::cout << "import shapefile successfully!" << std::endl;

        order = "dbname=" + database + " user=" + user + " password=" + password + " hostaddr=127.0.0.1 port=5432";
        pqxx::connection c(order);
        if (c.is_open()) {
        std::cout << "Connection succesful!" << std::endl;
        }
        else {
        std::cout << "Something went wrong... oops" << std::endl;
        }

        pqxx::work w(c);

        w.exec("ALTER TABLE jiugongge_road ADD COLUMN source integer,ADD COLUMN target integer;");
        w.exec("commit;");
        w.exec("ALTER TABLE jiugongge_road RENAME COLUMN geom TO the_geom;");
        w.exec("commit;");
        w.exec("SELECT pgr_createTopology('jiugongge_road', 0.000001, 'the_geom', 'gid');");
        w.exec("commit;");
        w.exec("ALTER TABLE jiugongge_road RENAME COLUMN shape_leng TO length;");
        w.exec("ALTER TABLE jiugongge_road alter  COLUMN  length  type double precision;");
        w.exec("ALTER TABLE jiugongge_road ALTER COLUMN the_geom TYPE geometry(linestring,4326) USING ST_GeometryN(the_geom, 1);");
        w.exec("commit;");
        std::cout << "topology created successfully!" << std::endl;
        return true;
    }
    catch(const std::exception& e)
    {
        std::cout << "some thing wrong when initial global map!" << std::endl;
        std::cerr << e.what() << '\n';
        return false;
    }


}

bool RoadManager::findReferenceRoad(double start_point_lon, double start_point_lat, double end_point_lon, double end_point_lat, bool is_normal)
{
    try
    {
        string database("changshu");
        string user("postgres");
        string password("postgres");
        string output("/tmp/shortest_road.csv");
        string order;

        order = "dbname=" + database + " user=" + user + " password=" + password + " hostaddr=127.0.0.1 port=5432";
        pqxx::connection c(order);
        if (c.is_open()) {
        std::cout << "Connection succesful!" << std::endl;
        }
        else {
        std::cout << "Something went wrong... oops" << std::endl;
        }

        pqxx::work w(c);

        if (not is_normal)
        {
            const char * sql_updateMap = nullptr;
            char updateOrder[] = "select pgr_updateTopoMap('jiugongge_road', %f, %f);";
            char updatebuf[strlen(updateOrder)];
            sprintf(updatebuf, updateOrder, start_point_lon, start_point_lat);
            sql_updateMap = updatebuf;
            w.exec(sql_updateMap);

            double blocked_point_lon = start_point_lon;
            double blocked_point_lat = start_point_lat;
            const char * sql_point_offset = nullptr;
            char offsetOrder[] = "select * from pgr_pointoffset('jiugongge_road', %f, %f);";
            char offsetbuf[strlen(offsetOrder)];
            sprintf(offsetbuf, offsetOrder, start_point_lon, start_point_lat);
            sql_point_offset = offsetbuf;
            pqxx::result R = w.exec(sql_point_offset);
            R[0][0].to(start_point_lon);
            R[1][0].to(start_point_lat);

            /*calculate the shortest path between start and end*/
            /**************************************************/
            const char *sql_shortPath = nullptr;
            // char sqlOrder[] = "select pgr_fromAtoB_unnormal('jiugongge_road', 'jiugongge_road_vertices_pgr', %f, %f, %f, %f, %f, %f) as pgr_fromatob into shortpath;";
            // char buf[strlen(sqlOrder)];
            // sprintf(buf, sqlOrder, blocked_point_lon, blocked_point_lat, start_point_lon, start_point_lat, end_point_lon, end_point_lat);
            char sqlOrder[] = "select pgr_fromAtoB('jiugongge_road', %f, %f, %f, %f) as pgr_fromatob into shortpath;";
            char buf[strlen(sqlOrder)];
            sprintf(buf, sqlOrder, start_point_lon, start_point_lat, end_point_lon, end_point_lat);
            sql_shortPath = buf;
            w.exec("drop table if exists shortpath;");
            w.exec(sql_shortPath);
            w.exec("commit;");
            std::cout << "shortpath created successfully!" << std::endl;
        }
        else
        {
            /*calculate the shortest path between start and end*/
            /**************************************************/
            const char *sql_shortPath = nullptr;
            char sqlOrder[] = "select pgr_fromAtoB('jiugongge_road', %f, %f, %f, %f) as pgr_fromatob into shortpath;";
            char buf[strlen(sqlOrder)];
            sprintf(buf, sqlOrder, start_point_lon, start_point_lat, end_point_lon, end_point_lat);
            sql_shortPath = buf;
            w.exec("drop table if exists shortpath;");
            w.exec(sql_shortPath);
            w.exec("commit;");
        }



        /*generate GPS path point in shortpath*/
        /**************************************************/

        pqxx::result R = w.exec("select pgr_fromatob is null from shortpath;");
        string is_path_null;
        R[0][0].to(is_path_null);
        if(is_path_null == "t")
        {
            std::cout << "error: shortest path is null!" << endl;
            return false;
        }
        else
        {
            std::cout << "shortpath created successfully!" << std::endl;
        }


        const char* sql_txt = nullptr;
        order = "select line2points('shortpath',0.5,'jiugongge_road_vertices_pgr');";
        sql_txt = order.c_str();
        w.exec(sql_txt);

        order = "drop table if exists point_in_buffer;"\
                "SELECT * into point_in_buffer FROM all_points "\
                "WHERE ST_Within(geom, (SELECT ST_Buffer((SELECT pgr_fromatob FROM shortpath),0.00001,'endcap=round join=round')));";
        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");

        order = "drop table if exists stop_point_in_buffer;"\
                "SELECT * into stop_point_in_buffer FROM stop_points "\
                "WHERE ST_Within(geom, (SELECT ST_Buffer((SELECT pgr_fromatob FROM shortpath),0.00001,'endcap=round join=round')));";
        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");
        std::cout<< "build buffer successfully!"<<std::endl;

        /*obtain property info by nearest search*/
        order = "drop table if exists result_path_points_meta;"\
                "SELECT p.id, p.lon, p.lat, b.heading, b.curv, b.mode, b.speed_mode, b.opposite_side_Mode, b.event_mode, b.lane_num, b.lane_seq, b.lane_width, p.point "\
                "into result_path_points_meta "\
                "FROM result_points_tbl p "\
                "CROSS JOIN LATERAL ( "\
                "SELECT * "\
                "FROM point_in_buffer r "\
                "ORDER BY r.geom <-> p.point "\
                "LIMIT 1 "\
                ")b;"\
                "drop table if exists result_points_tbl;";

        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");

        order = "drop table if exists stop_point_candidate;"
            "SELECT b.id "\
            "into stop_point_candidate "\
            "FROM stop_point_in_buffer p "\
            "CROSS JOIN LATERAL ( "\
            "SELECT * FROM result_path_points_meta r "\
            "ORDER BY p.geom <-> r.point LIMIT 1 )b; ";
        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");

        order = "update result_path_points_meta "\
                "set event_mode = 0 "\
                "where event_mode = 3;";
        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");

        order = "update result_path_points_meta a "\
                "set event_mode = 3 "\
                "from stop_point_candidate b "\
                "where a.id = b.id;";
        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");

        w.exec("drop table if exists result_path_points;");
        //w.exec("drop table if exists result_path_points_meta;");
        //w.exec("drop table if exists stop_point_candidate;");
        w.exec("commit;");
        w.exec("select p.id, p.lon, p.lat, p.heading, p.curv, p.mode, p.speed_mode, p.event_mode, p.opposite_side_Mode, p.lane_num, p.lane_seq, p.lane_width into result_path_points from result_path_points_meta p order by p.id;");
        w.exec("commit;");
        w.exec("drop table if exists stop_point_candidate");
        w.exec("drop table if exists result_path_points_meta;");
        w.exec("commit;");

        order = "COPY (SELECT * FROM result_path_points) TO '"+ output + "' (format csv, delimiter ' ');";
        sql_txt = order.c_str();
        w.exec(sql_txt);
        w.exec("commit;");

        readRoadMapFile(output);
        return true;
    }
    catch(const std::exception& e)
    {
        std::cout << "some thing wrong, when global planning!" << endl;
        std::cerr << e.what() << '\n';
        return false;
    }
}

void RoadManager::getLocalReferencePath(const NavInfo& car_position, vector<Point>& reference_road,
    vector<RoadInfo>& extra_informations, bool need_opposite){

    extra_informations.clear();
    reference_road.clear();
    const int search_depth = 200;
    const int search_history_depth = 50;
    double min_d = 1e10;
    const double angle_tolerance = 1.7;
    //nidx : rdmp_nearest_index
    bool global_nidx_just_updated = false;

    if(rdmp_neareast_index < 0){
        rdmp_neareast_index = getNearestPointIndex(car_position, road_map);
        global_nidx_just_updated = true;
        if(rdmp_neareast_index < 0)
            return;
    }

    int search_begin = max(rdmp_neareast_index - search_history_depth, 0);
    int search_end = min((int)road_map.size(), search_depth + search_begin);
    if(need_opposite) search_end = road_map.size();
    Point standard_point = car_position.current_position;
    standard_point.x = CAR_CEN_ROW;
    standard_point.y = CAR_CEN_COL;
    standard_point.angle.setByRad(0);
    int current_position_in_reference_road = 0;
    for(int i = search_begin; i < search_end; ++i){
        Point p = road_map[i];
        p.updateLocalCoordinate(standard_point);
        double dx = p.utmX - car_position.current_position.utmX;
        double dy = p.utmY - car_position.current_position.utmY;
        double da = fabs(car_position.current_position.heading.getRad() - p.heading.getRad());

        if(da < angle_tolerance){
            double dis = dx * dx + dy * dy;
            if(dis < min_d){
                min_d = dis;
                rdmp_neareast_index = i;
            }
        }

        if(!utils::isInLocalMap(p)){
            if(!need_opposite && reference_road.size() > 5) break;
            else continue;
        }

        if(i == rdmp_neareast_index)
            current_position_in_reference_road = reference_road.size() - 1;

        reference_road.push_back(p);
        extra_informations.push_back(road_map_information[i]);
    }

    if(reference_road.size() == 0){
        if(global_nidx_just_updated) return;
        //else try again
        //force to update the nearest index
        rdmp_neareast_index = -1;
        getLocalReferencePath(car_position, reference_road,
            extra_informations, need_opposite);
    }
    else{
        reference_road.front().s = 0;
		for(int i = 1; i < reference_road.size(); i++){
			double delta_s = hypot(fabs(reference_road[i].x - reference_road[i-1].x),
                fabs(reference_road[i].y - reference_road[i-1].y)) * GRID_RESOLUTION;
			reference_road[i].s = reference_road[i-1].s + delta_s;
		}

        double current_position_s = reference_road[current_position_in_reference_road].s;
        if(current_position_in_reference_road == reference_road.size() - 1)
            current_position_s = -hypot(
                fabs(reference_road[current_position_in_reference_road].x - standard_point.x),
                fabs(reference_road[current_position_in_reference_road].y - standard_point.y)
            );

        for(auto& p : reference_road) p.s -= current_position_s;
    }
}

void RoadManager::maintainPath(const NavInfo& car_position, const vector<Point>& path){
    //get current index: the nearest point in maintainedPath
	unique_lock<mutex> lck(maintained_path_mtx);
    maintained_path = path;
	if(!car_position.reliable) return;
    for(auto& point : maintained_path)
        point.updateGlobalCoordinate(car_position.current_position);
    maintained_path_published = false;
}

void RoadManager::maintainParkingLotList(const NavInfo& nav_info, ParkingLotList const& parking_lot_list){
	if(!nav_info.reliable) return;
	for(ParkingLot parking_lot: parking_lot_list.parking_lot_list){
		int replace_index = -1;
        double center_x = (parking_lot.left_back.x + parking_lot.right_front.x) / 2.0;
        double center_y = (parking_lot.left_back.y + parking_lot.right_front.y) / 2.0;
		int maintained_i = -1;
		for(auto& old_lot: maintained_parking_lot_list.parking_lot_list){
			maintained_i++;
        	old_lot.left_back.updateLocalCoordinate(nav_info.current_position);
        	old_lot.left_front.updateLocalCoordinate(nav_info.current_position);
        	old_lot.right_back.updateLocalCoordinate(nav_info.current_position);
        	old_lot.right_front.updateLocalCoordinate(nav_info.current_position);
        	double o_center_x = (old_lot.left_back.x + old_lot.right_front.x) / 2.0;
        	double o_center_y = (old_lot.left_back.y + old_lot.right_front.y) / 2.0;
			if(hypot(fabs(o_center_x - center_x), fabs(o_center_y - center_y)) > 2.0 / GRID_RESOLUTION) continue;
			replace_index = maintained_i;
		}
        parking_lot.left_back.updateGlobalCoordinate(nav_info.current_position);
        parking_lot.left_front.updateGlobalCoordinate(nav_info.current_position);
        parking_lot.right_back.updateGlobalCoordinate(nav_info.current_position);
        parking_lot.right_front.updateGlobalCoordinate(nav_info.current_position);
		if(replace_index >= 0){
			maintained_parking_lot_list.parking_lot_list[replace_index] = parking_lot;
		}else{
			maintained_parking_lot_list.parking_lot_list.push_back(parking_lot);
		}
	}
}

void RoadManager::getMaintainedParkingLotList(const NavInfo& nav_info, ParkingLotList& parking_lot_list){
	if(!nav_info.reliable) return;
    for(int i = 0; i < maintained_parking_lot_list.parking_lot_list.size(); ++i){
        ParkingLot& parking_lot = maintained_parking_lot_list.parking_lot_list[i];
        parking_lot.left_back.updateLocalCoordinate(nav_info.current_position);
        parking_lot.left_front.updateLocalCoordinate(nav_info.current_position);
        parking_lot.right_back.updateLocalCoordinate(nav_info.current_position);
        parking_lot.right_front.updateLocalCoordinate(nav_info.current_position);
        double center_x = (parking_lot.left_back.x + parking_lot.right_front.x) / 2.0;
        double center_y = (parking_lot.left_back.y + parking_lot.right_front.y) / 2.0;
        if(center_x < 0 || center_x >= GRID_ROW
            || center_y < 0 || center_y >= GRID_COL) continue;
		parking_lot_list.parking_lot_list.push_back(parking_lot);
    }
	if(parking_lot_list.parking_lot_list.size() > 0) parking_lot_list.detected = true;
}

int RoadManager::getNearestPointIndex(const NavInfo& car_position, const vector<Point>& road){
    double min_d = inf;
    int min_i = -1;
    const double angle_tolerance = 1.7;
    const double dis_tolerance = 400;
    double dx, dy, da, dis;
    for(int i = 0; i < road.size(); ++i){
        dx = car_position.current_position.utmX - road[i].utmX;
        dy = car_position.current_position.utmY - road[i].utmY;
        da = fabs(car_position.current_position.heading.getRad() - road[i].heading.getRad());
        if(da < angle_tolerance){
            dis = sqrt(dx*dx + dy*dy);
            if(dis < min_d){
                min_d = dis;
                min_i = i;
            }
        }
    }

    if(min_d >= dis_tolerance) return -1;
    return min_i;
}

void RoadManager::getMaintainedPath(const NavInfo& car_position, vector<Point>& maintained_path){
	unique_lock<mutex> lck(maintained_path_mtx);
    maintained_path.clear();
	if(!car_position.reliable){
    	maintained_path = this->maintained_path;
		return;
	}
    int nearest_point_idx = getNearestPointIndex(car_position, this->maintained_path);
    if(nearest_point_idx < 0) return; //maintained path is too far
    if(nearest_point_idx != 0)
        this->maintained_path.erase(this->maintained_path.begin(),
            this->maintained_path.begin() + nearest_point_idx);
	if(this->maintained_path.empty()) return;
	double start_s = this->maintained_path.front().s;
	double start_t = this->maintained_path.front().t;
    for(int i = 0; i < this->maintained_path.size(); ++i){
        this->maintained_path[i].s -= start_s;
        this->maintained_path[i].t -= start_t;
        this->maintained_path[i].updateLocalCoordinate(car_position.current_position);
    }
    maintained_path = this->maintained_path;
}

}
