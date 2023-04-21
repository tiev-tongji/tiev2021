
#ifndef CLOUD_SYSTEM_H
#define CLOUD_SYSTEM_H

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../proto/TJ_v2x.pb.h"
#include "decision_context.h"
#include "mqtt/client.h"
#include "mqtt/async_client.h"
#include "message_manager.h"

namespace TiEV {
const std::string SERVER_ADDRESS("tcp://221.181.123.244:22683");
const std::string CLIENT_ID("cloudpub_01");
const std::string TOPIC("cloudproc_02");
const std::string TOPIC_SEND("v2x/v1/obu/001/vsmdata/up");
const std::string USERNAME("cloudpub_01");
const std::string PASSWORD("Coudpub@pub_01!");
const int QOS = 1;
const int N_RETRY_ATTEMPTS = 5;
struct TrajectoryPose
{
  double x;
  double y;
  double theta;
  double v;
  double a;
  double relative_time;
};
struct C2Vinfo {
  double speed;
  double heading_angle;
  double utmX;
  double utmY;
};

struct C2VinfoTran {
  int      speed;
  int      heading_angle;
  int      utmX;
  int      utmY;
  uint64_t timestamp;
};
inline TrajectoryPoint getControlPathByTraj(const TrajectoryPose& pose,C2Vinfo& cur_car_info){
  Pose standard_point;
  standard_point.utm_position.utm_x = cur_car_info.utmX;
  standard_point.utm_position.utm_y = cur_car_info.utmY;
  standard_point.utm_position.heading = cur_car_info.heading_angle;
  double stdh = pose.theta - standard_point.utm_position.heading;
  double qx   = (pose.x - standard_point.utm_position.utm_x);
  double qy = (pose.y - standard_point.utm_position.utm_y) ;
  double sinstdh = sin(standard_point.utm_position.heading), cosstdh = cos(standard_point.utm_position.heading);
  double px = (qx * cosstdh - qy * sinstdh);
  double py = (qx * sinstdh + qy * cosstdh);

  TrajectoryPoint traj_point ;
  traj_point.x = px ;
  traj_point.y = py ;
  traj_point.theta = stdh;
  while(traj_point.theta > M_PI){
   traj_point.theta -= 1 * M_PI;
  }
  while(traj_point.theta <= -M_PI){
   traj_point.theta += 1 * M_PI;
  }
  traj_point.v = pose.v;
  traj_point.a = pose.a;
  traj_point.t = pose.relative_time; 

  return traj_point;
}


class action_listener : public virtual mqtt::iaction_listener
{
  std::string name_;

  void on_failure(const mqtt::token &tok) override
  {
    std::cout << name_ << " failure";
    if (tok.get_message_id() != 0)
      std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    std::cout << std::endl;
  }

  void on_success(const mqtt::token &tok) override
  {
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
      std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    auto top = tok.get_topics();
    if (top && !top->empty())
      std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
    std::cout << std::endl;
  }

 public:
  action_listener(const std::string &name) : name_(name) {}
};


class callback : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener

{
  int nretry_;
  mqtt::async_client &cli_;
  mqtt::connect_options &connOpts_;
  action_listener subListener_;

  void reconnect()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try
    {
      cli_.connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception &exc)
    {
      std::cerr << "Error: " << exc.what() << std::endl;
      exit(1);
    }
  }

  void on_failure(const mqtt::token &tok) override
  {
    std::cout << "Connection attempt failed" << std::endl;
    if (++nretry_ > N_RETRY_ATTEMPTS)
      exit(1);
    reconnect();
  }

  void on_success(const mqtt::token &tok) override {}

  void connected(const std::string &cause) override
  {
    std::cout << "\nConnection success" << std::endl;
    std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
              << "\tfor client " << CLIENT_ID
              << " using QoS" << QOS << "\n"
              << "\nPress Q<Enter> to quit\n"
              << std::endl;

    cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
  }

  void connection_lost(const std::string &cause) override
  {
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
      std::cout << "\tcause: " << cause << std::endl;

    std::cout << "Reconnecting..." << std::endl;
    nretry_ = 0;
    reconnect();
  }

  void message_arrived(mqtt::const_message_ptr msg) override
  {
    std::cout << "Message arrived" << std::endl;
    std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
    cn::seisys::v2x::pb::CloudToV cloudToV;

    cloudToV.ParseFromString(msg->get_payload());
    u_int64_t traj_timestamp = cloudToV.timestamp();
    std::cout << "timestamp: " << traj_timestamp << std::endl;
    auto C2V_planning = cloudToV.pathplanning();
    std::unique_ptr<std::vector<TrajectoryPose>> traj_from_cloud(new std::vector<TrajectoryPose>);
    for (cn::seisys::v2x::pb::PathPlanningPoint &point_tmp : C2V_planning)
    {
      TrajectoryPose pose_tmp;
      //std::cout << "acceleration: " << point_tmp.acceleration(). << std::endl;
      std::cout << "--------------------------------------------\n"<< std::endl;
      pose_tmp.x = 1e-2 * point_tmp.pos().lat();
      pose_tmp.y = 1e-2 * point_tmp.pos().lon();
      pose_tmp.theta = 0.0125 * M_PI / 180.0 * point_tmp.heading();
      pose_tmp.v = 0.02 * point_tmp.speed();
      pose_tmp.relative_time = 0.01 * point_tmp.estimatedtime();

      std::cout <<" the point is " << pose_tmp.x << " "<< pose_tmp.y  <<" with heading  " << pose_tmp.theta << std::endl;
      traj_from_cloud->push_back(pose_tmp);
    }
    std::cout << "Trajectory Size: " << traj_from_cloud->size() << std::endl;
    std::cout <<"--------------------------------------------\n"<< std::endl;


    structAIMPATH control_path;
    NavInfo cur_nav;
    auto& mesg_manager = MessageManager::getInstance();
    if (mesg_manager.getNavInfo(cur_nav) == 0) {
      std::cerr << "mesg is unable to get navinfo" << std::endl;
    }
    auto cur_car = cur_nav.car_pose;
    C2Vinfo cur_car_info = {cur_car.v, cur_car.utm_position.heading, cur_car.utm_position.utm_x,
                          cur_car.utm_position.utm_y};

    control_path.num_points = traj_from_cloud->size();
    for( auto& pose : (*traj_from_cloud) ){      

      auto traj_point = getControlPathByTraj(pose,cur_car_info);

      std::cout <<" the control point is " << traj_point.x <<"," << traj_point.y <<" : " << traj_point.theta<< std::endl;
      control_path.points.push_back(traj_point);
    }

    mesg_manager.publishPath(control_path);
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

 public:
  callback(mqtt::async_client &cli, mqtt::connect_options &connOpts)
      : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}
};



void sendToCloud(const C2Vinfo &info,mqtt::async_client& client_);

}  // namespace TiEV




#endif  // CLOUD_SYSTEM_H