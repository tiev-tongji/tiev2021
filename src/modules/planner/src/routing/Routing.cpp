#include "Routing.h"
#include "message_manager.h"
#include "tiev_class.h"
#include "decision.h"

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
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using routing_service::RefRoad;
using routing_service::RefRoadPoint;
using routing_service::RoutingService;
using routing_service::TaskPoints;
using routing_service::MapService;
using grpc::ClientWriter;
using google::protobuf::Empty;
using routing_service::CarInfo;
using routing_service::TaskPoint;
Routing::Routing() {

    host        = Config::getInstance()->host;
    port        = Config::getInstance()->port;
    dbname      = Config::getInstance()->dbname;
    user        = Config::getInstance()->user;
    password    = Config::getInstance()->password;
    connect_sql = "dbname=" + dbname + " user=" + user + " password=" + password + " hostaddr=" + host + " port=" + port;
    output      = Config::getInstance()->output;
    topo_name   = Config::getInstance()->topo_name;
    std::cout<<"target url: "<<host+":"+port<<std::endl;
    shared_ptr<Channel> channel = grpc::CreateChannel(host+":"+port, grpc::InsecureChannelCredentials());
    stub        = unique_ptr<RoutingService::Stub>(RoutingService::NewStub(channel));
    map_stub    = unique_ptr<MapService::Stub>(MapService::NewStub(channel));
    writer      = unique_ptr<ClientWriter<CarInfo>>(map_stub->UpdateCarInfo(&writer_context, &writer_empty));
}

Routing::~Routing() {}

int Routing::findReferenceRoad(std::vector<HDMapPoint>& global_path, const std::vector<Task>& task_points, bool blocked) {
    global_path.clear();
    if(task_points.size() < 2) {
        std::cerr << "not enough points" << std::endl;
        return -1;
    }
    //调用service，处理返回结果
    TaskPoints request;
    request.set_blocked(blocked);
    for(const auto& p : task_points) {
        auto point = request.add_task_point();
        point->set_lon(p.lon_lat_position.lon);
        point->set_lat(p.lon_lat_position.lat);
    }
    //指定使用的地图
    request.set_map(dbname);
    ClientContext context;
    RefRoad       response;
    Status        status = stub->FindReferenceRoad(&context, request, &response);
    if(status.ok()) {
        std::cout << "RPC success" << std::endl;
        int    sum_costs      = response.time_cost();
        size_t sum_points_num = response.point_size();
        for(size_t i = 0; i < sum_points_num; i++) {
            auto       res_point = response.point(i);
            HDMapPoint p;
            p.utm_position = UtmPosition(res_point.utmx(), res_point.utmy(), res_point.heading());
            p.mode         = HDMapMode(res_point.mode());
            p.speed_mode   = HDMapSpeed(res_point.speed_mode());
            p.event        = HDMapEvent(res_point.event_mode());
            p.block_type   = BlockType(res_point.opposite_side_mode());
            p.lane_num     = res_point.lane_num();
            p.lane_seq     = res_point.lane_seq();
            p.lane_width   = res_point.lane_width();
            global_path.push_back(p);
        }
        std::cout << "Build reference line file sucessfully which is " << output << std::endl;
        std::cout << "total points number: " << sum_points_num << std::endl;
        std::cout << "estimated time cost: " << sum_costs << "s, " << sum_costs / 60 << "mins" << std::endl;
        return sum_costs;
    }
    else {
        std::cerr << "RPC failed" << std::endl;
        std::cerr<< "error code: "<<status.error_code()<<std::endl;
        std::cerr<<"error message: "<<status.error_message()<<std::endl;
        std::cerr<<"error detail: "<<status.error_details()<<std::endl;
        return -1;
    }
}

void Routing::Array2Str(const std::vector<Task>& task_points, std::string& array_x_str, std::string& array_y_str) {
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

void Routing::updateInfoToServer(){
    CarInfo info;
    info.set_map(dbname);
    auto pos = info.mutable_pos();
    MessageManager* msg_m = MessageManager::getInstance();
    NavInfo nav_info;
    msg_m->getNavInfo(nav_info);
    pos->set_lon(nav_info.lon);
    pos->set_lat(nav_info.lat);
    MachineManager* mm = MachineManager::getInstance();
    if(mm->machine.isActive<GlobalPlanning>()||mm->machine.isActive<TemporaryStop>()){
        info.set_running(false);
    }else{
        info.set_running(true);
    }
    if(!writer->Write(info)){
        std::cerr<<"can't update car info to server!"<<std::endl;
    }
} 

Task Routing::waitForNextTask(){
    Empty empty;
    ClientContext context;
    TaskPoint res;
    Task result;
    std::cout<<"Waiting for next task..."<<std::endl;
    Status status = map_stub->WaitForTaskPoint(&context, empty, &res);
    if(status.ok()){
        result.lon_lat_position.lon = res.lon();
        result.lon_lat_position.lat = res.lat();
        result.utm_position.utm_x = res.utmx();
        result.utm_position.utm_y = res.utmy();
        result.on_or_off = res.on_or_off();
        result.task_points.clear();
        result.task_points.push_back(UtmPosition(res.utmx(), res.utmy(), res.heading()));
    }else{
        std::cerr<<"can't get task from server, error message: "<<status.error_message()<<std::endl;
    }
    std::cout<<"Get next task"<<std::endl;
    return result;
}  
}