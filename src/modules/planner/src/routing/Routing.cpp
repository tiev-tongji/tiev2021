#include "Routing.h"

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include "config.h"
#include "tiev_class.h"
#include "tiev_utils.h"
#include "tievlog.h"

// #include "angle.h"
// #include "coordinate_converter/coordinate_converter.h"

using google::protobuf::Empty;
using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientWriter;
using grpc::Status;
using routing_service::CarInfo;
using routing_service::MapService;
using routing_service::RefRoad;
using routing_service::RefRoadPoint;
using routing_service::RoutingService;
using GRPC_TaskPoint  = routing_service::TaskPoint;
using GRPC_TaskPoints = routing_service::TaskPoints;
namespace TiEV {
Routing::Routing() {
  host        = Config::getInstance().host;
  port        = Config::getInstance().port;
  dbname      = Config::getInstance().dbname;
  user        = Config::getInstance().user;
  password    = Config::getInstance().password;
  connect_sql = "dbname=" + dbname + " user=" + user + " password=" + password +
                " hostaddr=" + host + " port=" + port;
  output    = Config::getInstance().output;
  topo_name = Config::getInstance().topo_name;
  LOG(INFO) << "target url: " << host + ":" + port;
  std::shared_ptr<Channel> channel = grpc::CreateChannel(
      host + ":" + port, grpc::InsecureChannelCredentials());
  stub =
      std::unique_ptr<RoutingService::Stub>(RoutingService::NewStub(channel));
  map_stub = std::unique_ptr<MapService::Stub>(MapService::NewStub(channel));
  writer   = std::unique_ptr<ClientWriter<CarInfo>>(
      map_stub->UpdateCarInfo(&writer_context, &writer_empty));
}

Routing::~Routing() {}

int Routing::findReferenceRoad(std::vector<HDMapPoint>&      global_path,
                               const std::vector<TaskPoint>& task_points,
                               bool                          blocked) {
  LOG(INFO) << "Find Road by Requesting Remote Service...";
  const auto start_time = getTimeStamp();
  global_path.clear();
  if (task_points.size() < 2) {
    LOG(WARNING) << "Please set more than 2 task points";
    return -1;
  }
  //调用service，处理返回结果
  GRPC_TaskPoints request;
  request.set_blocked(blocked);
  for (const auto& p : task_points) {
    auto point = request.add_task_point();
    point->set_lon(p.lon);
    point->set_lat(p.lat);
    point->set_utm_x(p.utm_x);
    point->set_utm_y(p.utm_y);
    point->set_heading(p.heading);
  }
  //指定使用的地图
  request.set_map(dbname);
  ClientContext context;
  RefRoad       response;
  Status        status = stub->FindReferenceRoad(&context, request, &response);
  if (status.ok()) {
    LOG(INFO) << "Remote Service Connection Success!";
    int    sum_costs      = response.time_cost();
    size_t sum_points_num = response.point_size();
    switch (sum_costs) {
      case -1:
      case -3:
        LOG(WARNING) << "Remote Service Error!";
        return sum_costs;
      case -2:
        LOG(WARNING) << "Car is far from map topology!";
        return sum_costs;
      case -4:
        LOG(INFO) << "No need to change Global Path!";
        return sum_costs;
    }
    for (size_t i = 0; i < sum_points_num; i++) {
      auto       res_point = response.point(i);
      HDMapPoint p;
      p.utm_position =
          UtmPosition(res_point.utmx(), res_point.utmy(), res_point.heading());
      p.mode                     = HDMapMode(res_point.mode());
      p.speed_mode               = HDMapSpeed(res_point.speed_mode());
      p.event                    = HDMapEvent(res_point.event_mode());
      p.block_type               = BlockType(res_point.opposite_side_mode());
      p.lane_num                 = res_point.lane_num();
      p.lane_seq                 = res_point.lane_seq();
      p.lane_width               = res_point.lane_width();
      p.lon_lat_position.lon     = res_point.lon();
      p.lon_lat_position.lat     = res_point.lat();
      p.lon_lat_position.heading = res_point.heading();
      global_path.push_back(p);
    }
    // LOG(INFO) << "requst routing server time: "
    //              << (getTimeStamp() - start_time) * 1e-6 << "s";
    LOG(INFO) << "Estimated time cost: " << sum_costs / 60 << "mins "
              << sum_costs % 60 << "s";
    return sum_costs;
  } else {
    LOG(WARNING) << "Find Error code: " << status.error_code()
                 << " Error message: " << status.error_message();
    return -1;
  }
}

int Routing::requestUpdateReferenceRoad(const TaskPoint& start_task_point,
                                        std::vector<HDMapPoint>* global_path) {
  //调用service，处理返回结果
  GRPC_TaskPoints request;
  auto*           point = request.add_task_point();
  point->set_lon(start_task_point.lon);
  point->set_lat(start_task_point.lat);
  point->set_utm_x(start_task_point.utm_x);
  point->set_utm_y(start_task_point.utm_y);
  point->set_heading(start_task_point.heading);
  //指定使用的地图
  request.set_map(dbname);
  ClientContext context;
  RefRoad       response;
  LOG(INFO) << "Update Road in Remote...";
  Status status =
      stub->RequestUpdateReferenceRoad(&context, request, &response);
  if (status.ok()) {
    LOG(INFO) << "Request success!";
    int sum_costs = response.time_cost();
    LOG(INFO) << "Estimated time cost: " << sum_costs / 60 << "mins "
              << sum_costs % 60;
    switch (sum_costs) {
      case -1:
      case -3:
        LOG(WARNING) << "Remote Service Error!";
        return sum_costs;
      case -2:
        LOG(WARNING) << "Car is far from map topology!";
        return sum_costs;
      case -4:
        LOG(INFO) << "No need to change Global Path!";
        return sum_costs;
    }
    size_t size = response.point_size();
    // std::cout << "id lon lat utmx utmy heading curv mode speed_mode "
    //              "event_mode opposite_side_mode lane_num lane_seq lane_width"
    //           << std::endl;
    for (size_t i = 0; i < size; i++) {
      auto       res_point = response.point(i);
      HDMapPoint p;
      p.utm_position =
          UtmPosition(res_point.utmx(), res_point.utmy(), res_point.heading());
      p.mode                     = HDMapMode(res_point.mode());
      p.speed_mode               = HDMapSpeed(res_point.speed_mode());
      p.event                    = HDMapEvent(res_point.event_mode());
      p.block_type               = BlockType(res_point.opposite_side_mode());
      p.lane_num                 = res_point.lane_num();
      p.lane_seq                 = res_point.lane_seq();
      p.lane_width               = res_point.lane_width();
      p.lon_lat_position.lon     = res_point.lon();
      p.lon_lat_position.lat     = res_point.lat();
      p.lon_lat_position.heading = res_point.heading();
      global_path->push_back(p);
    }
    return sum_costs;
  } else {
    LOG(WARNING) << "Update Error code: " << status.error_code()
                 << " Error message: " << status.error_message();
    return -1;
  }
}

void Routing::Array2Str(const std::vector<TaskPoint>& task_points,
                        std::string& array_x_str, std::string& array_y_str) {
  array_x_str.append("ARRAY[");
  array_y_str.append("ARRAY[");
  for (auto it = task_points.begin(); it != task_points.end(); ++it) {
    double lon = it->lon;
    double lat = it->lat;
    if (it != task_points.begin()) {
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

void Routing::updateInfoToServer() {
  //   CarInfo info;
  //   info.set_map(dbname);
  //   auto            pos   = info.mutable_pos();
  //   MessageManager* msg_m = MessageManager::getInstance();
  //   NavInfo         nav_info;
  //   msg_m->getNavInfo(nav_info);
  //   pos->set_lon(nav_info.lon);
  //   pos->set_lat(nav_info.lat);
  //   MachineManager* mm = MachineManager::getInstance();
  //   if (mm->machine.isActive<GlobalPlanning>() ||
  //       mm->machine.isActive<TemporaryStop>()) {
  //     info.set_running(false);
  //   } else {
  //     info.set_running(true);
  //   }
  //   if (!writer->Write(info)) {
  //     std::cerr << "can't update car info to server!" << std::endl;
  //   }
}

/*
Task Routing::waitForNextTask() {
  Empty          empty;
  ClientContext  context;
  GRPC_TaskPoint res;
  Task           result;
  std::cout << "Waiting for next task..." << std::endl;
  Status status = map_stub->WaitForTaskPoint(&context, empty, &res);
  if (status.ok()) {
    result.lon_lat_position.lon = res.lon();
    result.lon_lat_position.lat = res.lat();
    result.utm_position.utm_x   = res.utmx();
    result.utm_position.utm_y   = res.utmy();
    result.get_on               = !res.on_or_off();
    result.task_points.clear();
    result.task_points.push_back(
        UtmPosition(res.utmx(), res.utmy(), res.heading()));
  } else {
    std::cerr << "can't get task from server, error message: "
              << status.error_message() << std::endl;
  }
  std::cout << "Get next task" << std::endl;
  return result;
}
*/
}  // namespace TiEV
