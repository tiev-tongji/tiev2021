#include "Routing.h"

#include "config.h"
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <pqxx/pqxx>
#include <sstream>
#include <string>

// #include "angle.h"
// #include "coordinate_converter/coordinate_converter.h"

namespace TiEV {
using namespace std;
Routing::Routing() {

    host        = Config::getInstance()->host;
    port        = Config::getInstance()->host;
    dbname      = Config::getInstance()->host;
    user        = Config::getInstance()->host;
    password    = Config::getInstance()->host;
    connect_sql = "dbname=" + dbname + " user=" + user + " password=" + password + " hostaddr=" + host + " port=" + port;
    output      = Config::getInstance()->output;
    topo_name   = Config::getInstance()->topo_name;
}

Routing::~Routing() {}

int Routing::findReferenceRoad(std::vector<HDMapPoint>& global_path, const std::vector<TaskPoint>& task_points, bool blocked) {
    if(task_points.size() < 2) {
        std::cout << "task points size < 2 !!!" << std::endl;
        return -1;
    }

    try {
        std::cout << connect_sql << std::endl;
        pqxx::connection c(connect_sql.c_str());
        if(c.is_open()) {
            std::cout << "Connection succesful!" << std::endl;
        }
        else {
            std::cout << "Opps..something went wrong when connecting! check your config or database." << std::endl;
        }
        pqxx::work w(c);
        bool       is_blocked = blocked;
        std::vector<pqxx::result, std::allocator<pqxx::result>> results_vec;  // 查询记录
        int sum_costs      = 0;                                               //总花费时间
        int sum_points_num = 0;
        for(size_t start_idx = 0; start_idx < task_points.size() - 1; ++start_idx) {
            double start_lon = task_points[start_idx].lon_lat_position.lon;
            double start_lat = task_points[start_idx].lon_lat_position.lat;
            double end_lon   = task_points[start_idx + 1].lon_lat_position.lon;
            double end_lat   = task_points[start_idx + 1].lon_lat_position.lat;

            if(!is_blocked) {
                // 正常规划
                // std::string array_lon_str;
                // std::string array_lat_str;
                // Array2Str(task_points, array_lon_str, array_lat_str);
                // 创建新表shortpath, 字段名为 pgr_fromatontob
                char sql[] = "select pgr_fromAtoB('%s', %lf, %lf, %lf, %lf) as pgr_fromatontob into shortpath;";
                char sql_shortPath[200];
                sprintf(sql_shortPath, sql, topo_name.c_str(), start_lon, start_lat, end_lon, end_lat);
                std::cout << sql_shortPath << std::endl;
                w.exec("drop table if exists shortpath;");
                w.exec(sql_shortPath);
                w.exec("commit;");
            }
            else {
                is_blocked = false;  //第一段路径执行调头规划后, 后面的路段规划不再执行.
                // 前方道路堵塞, 调头规划
                std::cout << "road blocked! begin re-routing..." << std::endl;
                double blocked_point_lon = task_points[0].lon_lat_position.lon;
                double blocked_point_lat = task_points[0].lon_lat_position.lat;

                const char* sql_updateMap = nullptr;
                char        updateOrder[] = "select pgr_updateTopoMap('%s', %f, %f);";
                char        updatebuf[strlen(updateOrder)];
                sprintf(updatebuf, updateOrder, topo_name.c_str(), blocked_point_lon, blocked_point_lat);
                sql_updateMap = updatebuf;
                w.exec(sql_updateMap);

                const char* sql_point_offset = nullptr;
                char        offsetOrder[]    = "select * from pgr_pointoffset('%s', %f, %f);";
                char        offsetbuf[strlen(offsetOrder)];
                sprintf(offsetbuf, offsetOrder, topo_name.c_str(), blocked_point_lon, blocked_point_lat);
                sql_point_offset = offsetbuf;
                pqxx::result R   = w.exec(sql_point_offset);

                double new_start_lon;
                double new_start_lat;
                R[0][0].to(new_start_lon);
                R[1][0].to(new_start_lat);
                // task_points[0] = std::make_pair(new_start_lon, new_start_lat);
                std::cout << "start point offset to: lon " << new_start_lon << " lat " << new_start_lat << std::endl;

                // std::string array_x_str;
                // std::string array_y_str;
                // Array2Str(task_points, array_x_str, array_y_str);
                char sql[] = "select pgr_fromAtoB('%s', %lf, %lf, %lf, %lf) as pgr_fromatontob into shortpath;";
                char sql_shortPath[200];
                sprintf(sql_shortPath, sql, topo_name.c_str(), new_start_lon, new_start_lat, end_lon, end_lat);
                std::cout << sql_shortPath << std::endl;
                w.exec("drop table if exists shortpath;");
                w.exec(sql_shortPath);
                w.exec("commit;");
            }

            //搜索规划路径shortpath附近的参考路点
            pqxx::result R = w.exec("select pgr_fromatontob is null from shortpath;");
            std::string  is_path_null;
            R[0][0].to(is_path_null);
            if(is_path_null == "t") {
                std::cout << "error: shortest path is null!" << std::endl;
                return -1;
            }
            else {
                std::cout << "Create shortpath successfully!" << std::endl;
            }

            //搜索位于最短路径上的参数路属性点
            w.exec("select line2points('shortpath');");

            // 获取路点到本地文件
            // std::string copy_cmd = "psql -h " + host + " -p " + port + " -U " + user +" -d " + dbname +" -w -c " +
            //                         "\"\\copy result_path_points to " + output + " with csv header delimiter ' ';\"";
            // system(copy_cmd.c_str());
            pqxx::result path_points_result = w.exec("select * from result_path_points");
            sum_points_num += path_points_result.affected_rows();
            std::cout << "points num: " << path_points_result.affected_rows() << std::endl;
            results_vec.push_back(path_points_result);

            char cal_cost_sql[50];
            sprintf(cal_cost_sql, "select calculate_cost('%s');", topo_name.c_str());
            pqxx::result r_cost    = w.exec(cal_cost_sql);
            int          time_cost = (int)r_cost[0][0].as<float>();
            sum_costs += time_cost;
        }  // end for

        // save ref road points in file
        std::ofstream fout(output);
        fout << "id lon lat utmx utmy heading curv mode speed_mode "
                "event_mode opposite_side_mode lane_num lane_seq lane_width"
             << std::endl;
        for(auto result : results_vec) {
            for(auto row : result) {
                HDMapPoint p;
                p.utm_position = UtmPosition(stod(row[3].as<string>()), stod(row[4].as<string>()), stod(row[5].as<string>()));
                p.mode         = HDMapMode(stoi(row[7].as<string>()));
                p.speed_mode   = HDMapSpeed(stoi(row[8].as<string>()));
                p.event        = HDMapEvent(stoi(row[9].as<string>()));
                p.block_type   = BlockType(stoi(row[10].as<string>()));
                p.lane_num     = stoi(row[11].as<string>());
                p.lane_seq     = stoi(row[12].as<string>());
                p.lane_width   = stod(row[13].as<string>());
                global_path.emplace_back(p);
            }
        }
        fout.close();
        std::cout << "Build reference line file sucessfully which is " << output << std::endl;
        std::cout << "total points number: " << sum_points_num << std::endl;
        std::cout << "estimated time cost: " << sum_costs << "s, " << sum_costs / 60 << "mins" << std::endl;
        return sum_costs;
    }
    catch(pqxx::sql_error const& e) {
        std::cout << "Something went wrong when global planning!" << std::endl;
        std::cerr << "SQL error: " << e.what() << std::endl;
        std::cerr << "Query was: " << e.query() << std::endl;
        return -1;
    }
    catch(const std::exception& e) {
        std::cout << "Something went wrong when global planning!" << std::endl;
        std::cout << e.what() << std::endl;
        return -1;
    }
}

void Routing::Array2Str(const std::vector<TaskPoint>& task_points, std::string& array_x_str, std::string& array_y_str) {
    array_x_str.append("ARRAY[");
    array_y_str.append("ARRAY[");
    for(auto it = task_points.begin(); it != task_points.end(); ++it) {
        double lon = it->lon_lat_position.lon;
        double lat = it->lon_lat_position.lat;
        if(it != task_points.begin()) {
            array_x_str.append(", ");
            array_y_str.append(", ");
        }
        array_x_str.append(std::to_string(lon));
        array_y_str.append(std::to_string(lat));
    }
    array_x_str.append("]");
    array_y_str.append("]");
    // std::cout << array_x_str << std::endl << array_y_str << std::endl;
}
}