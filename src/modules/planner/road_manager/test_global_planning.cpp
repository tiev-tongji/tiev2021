//#include "road_manager.h"
//#include "config/config.h"
#include <iostream>
#include <pqxx/pqxx>
#include <stdlib.h>
#include <stdio.h>
#include <string>

using namespace std;

bool findReferenceRoad(double start_point_lon, double start_point_lat, double end_point_lon, double end_point_lat, bool is_normal)
{
    string database("gis");
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
        return false;
    }
    else
    {
      std::cout << "shortpath created successfully!" << std::endl;
    }

    //w.exec("ALTER TABLE shortpath ALTER COLUMN pgr_fromatob TYPE geometry(linestring,4326) USING ST_GeometryN(pgr_fromatob, 1);");
    //w.exec("commit;");
    

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

    /*update table to filte wrong property especially stop point*/
    // order = "drop table if exists result_path_points_meta2;"
    //         "SELECT p.id, p.lon, p.lat, p.heading, p.curv, p.mode, p.speed_mode, p.event_mode, p.lane_num, p.lane_seq, p.lane_width, p.point, b.geom "\
    //         "into result_path_points_meta2 "\
    //         "FROM result_path_points_meta p "\
    //         "CROSS JOIN LATERAL ( "\
    //         "SELECT * FROM stop_points r "\
		//         "ORDER BY r.geom <-> p.point LIMIT 1 )b; ";
    // sql_txt = order.c_str();
    // w.exec(sql_txt);
    // w.exec("commit;");

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

    //w.exec("select * into result_point_updated from result_path_points_meta p order by p.id;");

    // order = "update result_path_points_meta2 "\
    //         "set event_mode = 3 "\
    //         "where ST_DistanceSpheroid(point, geom, 'SPHEROID[\"WGS 84\",6378137,298.257223563]') < 1;";
    // sql_txt = order.c_str();
    // w.exec(sql_txt);
    // w.exec("commit;");
    // std::cout<<"update success!"<<std::endl;

    w.exec("drop table if exists result_path_points;");
    w.exec("commit;");
    w.exec("select p.id, p.lon, p.lat, p.heading, p.curv, p.mode, p.speed_mode, p.event_mode, p.opposite_side_Mode, p.lane_num, p.lane_seq, p.lane_width into result_path_points from result_path_points_meta p order by p.id;");
    w.exec("commit;");
    //w.exec("drop table if exists result_path_points_meta;");
    //w.exec("drop table if exists result_path_points_meta2;");
    w.exec("commit;");

    order = "COPY (SELECT * FROM result_path_points) TO '"+ output + "' (format csv, delimiter ' ');";
    sql_txt = order.c_str();
    w.exec(sql_txt);
    w.exec("commit;");
    //exit_nicely(c)

    //readRoadMapFile(output);
    return true;
}


void initGlobalMap(string shp_path)
{
    string database("gis");
    string user("postgres");
    string password("postgres");
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
    w.exec("drop table if exists jiugongge_road;");
    w.exec("commit;");


    order = "shp2pgsql -s 4326 -d " + shp_path + " jiugongge_road |psql -h 127.0.0.1 -p 5432 -U " + user + " -d " + database;
    system(order.c_str());
    std::cout << "import shapefile successfully!" << std::endl;

    
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
}


int main()
{
    //Config const *cfg = Config::getInstance();
    //RoadManager *road_manager = RoadManager::getInstance();
	//road_manager->readRoadMapFile(cfg->roadmap_file);
  //120.780431, 31.593236  120.774337, 31.589887  120.771458, 31.593326 120.778133,31.594398 120.773225021,31.5928278745
  //g++ -std=gnu++0x test_global_planning.cpp -lpqxx -lpq -o findPath
  //120.772816555, 31.5925309738  120.7806496,31.5931118
  try
  {
    //initGlobalMap("/home/ljf/tiev2019/env/postgres/map/changshu/1113/jiugongge.shp");
    initGlobalMap("/home/ljf/tiev2019/env/postgres/map/v1/total_map_v1.shp");
    //initGlobalMap("/home/ljf/tiev2019/env/postgres/map/changshu/topo_edited_1107_3/topo_edited_1107_3.shp");
    findReferenceRoad(120.772816555, 31.5925309738,  120.7806496,31.5931118, true);
  }
  catch(const std::exception& e)
  {
    std::cout << "some things wrong! when global planning" << std::endl;
    std::cerr << e.what() << '\n';
  }
  

}
