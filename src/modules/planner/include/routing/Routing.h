#ifndef ROUTING_H
#define ROUTING_H

#include "config.h"
#include "pose.h"
#include <map>
#include <string>
#include <vector>

/***********
 * 一次规划
***********/

namespace TiEV {
static mutex routing_mtx;
class Routing {
public:
    static Routing* getInstance() {
        routing_mtx.lock();
        static Routing instance;
        routing_mtx.unlock();
        return &instance;
    }

    /**
    * @brief Find the reference line according to start and end point.
    *        result file is stored in /tmp/shortest_road_tlp.csv
    * @param start_point_lon
    * @param start_point_lat
    * @param end_point_lon
    * @param end_point_lat
    * @param  is_normal (true means the normal situation that there is no blocked node)
    * @return true means that build result file sucessfully.
    * @return false means that something went wrong
    */
    // static bool findReferenceRoad(double start_point_lon, double start_point_lat, double end_point_lon, double end_point_lat, bool is_normal = true);

    /**
     * one time routing
     *
     * @param task_points:任务点(起点, 任务点1, 任务点2, ..., 终点) utm坐标点集
     * @param block是否遇到阻塞,选择掉头
     *
     * @return 返回预估完成任务时间, 单位秒s, -1代表寻路失败.
     */
    int findReferenceRoad(std::vector<HDMapPoint>& global_path, const std::vector<TaskPoint>& task_points, bool blockeds = false);

private:
    Routing();
    ~Routing();

    std::string host;
    std::string port;
    std::string dbname;
    std::string user;
    std::string password;
    std::string output;
    std::string connect_sql;
    std::string topo_name;

    //将输入的task points转为sql array语句
    void Array2Str(const std::vector<TaskPoint>& task_points, std::string& array_x_str, std::string& array_y_str);
};
}
#endif