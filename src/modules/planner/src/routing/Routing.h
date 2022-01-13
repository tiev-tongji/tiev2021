#ifndef ROUTING_H
#define ROUTING_H

#include <grpcpp/grpcpp.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "config.h"
#include "pose.h"
#include "routing_service.grpc.pb.h"

/***********
 * 一次规划
 ***********/

namespace TiEV {
using google::protobuf::Empty;
using grpc::ClientContext;
using grpc::ClientWriter;
using routing_service::CarInfo;
static mutex routing_mtx;
class Routing {
 public:
 public:
  Routing(const Routing&) = delete;
  Routing& operator=(const Routing&) = delete;

  static Routing& getInstance() {
    static Routing instance;
    return instance;
  }

  /**
   * @brief Find the reference line according to start and end point.
   *        result file is stored in /tmp/shortest_road_tlp.csv
   * @param start_point_lon
   * @param start_point_lat
   * @param end_point_lon
   * @param end_point_lat
   * @param  is_normal (true means the normal situation that there is no blocked
   * node)
   * @return true means that build result file sucessfully.
   * @return false means that something went wrong
   */
  // static bool findReferenceRoad(double start_point_lon, double
  // start_point_lat, double end_point_lon, double end_point_lat, bool is_normal
  // = true);

  /**
   * one time routing
   *
   * @param task_points:任务点(起点, 任务点1, 任务点2, ..., 终点) utm坐标点集
   * @param block是否遇到阻塞,选择掉头
   *
   * @return 返回预估完成任务时间, 单位秒s, -1代表寻路失败.
   */
  int findReferenceRoad(std::vector<HDMapPoint>& global_path,
                        const std::vector<Task>& task_points,
                        bool                     blockeds = false);

  int requestUpdateReferenceRoad(const LonLatPosition& start_lon_lat_position,
                                 const UtmPosition&    start_utm_position,
                                 std::vector<HDMapPoint>* global_path);

  // 将车辆信息发送至服务器
  void updateInfoToServer();
  // 获取下一个任务点
  Task waitForNextTask();

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

  // gRPC stubs for client
  std::unique_ptr<routing_service::RoutingService::Stub> stub;
  std::unique_ptr<routing_service::MapService::Stub>     map_stub;

  // arguments for update car info
  ClientContext                          writer_context;
  Empty                                  writer_empty;
  std::unique_ptr<ClientWriter<CarInfo>> writer;

  //将输入的task points转为sql array语句
  void Array2Str(const std::vector<Task>& task_points, std::string& array_x_str,
                 std::string& array_y_str);
};
}  // namespace TiEV
#endif