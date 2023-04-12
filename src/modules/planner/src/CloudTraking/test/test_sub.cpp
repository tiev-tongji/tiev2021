#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <cmath>
#include "mqtt/async_client.h"
#include "../proto/TJ_v2x.pb.h"


struct TrajectoryPose
{
  double x;
  double y;
  double theta;
  double v;
  double a;
  double relative_time;
};


const std::string SERVER_ADDRESS("tcp://221.181.123.244:22683");
const std::string CLIENT_ID("cloudpub_03");
const std::string TOPIC("cloudproc_02");
const std::string USERNAME("cloudpub_03");
const std::string PASSWORD("Coudpub@pub_03!");
const int QOS = 1;
const int N_RETRY_ATTEMPTS = 5;

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
    // std::cout << "\tpayload: '" << msg->to_string()<< "'\n"<< std::endl;

    cn::seisys::v2x::pb::CloudToV cloudToV;

    // cn::seisys::v2x::pb::Position3D position3D;
    // cn::seisys::v2x::pb::ObstacleData obstacleData;
    cloudToV.ParseFromString(msg->get_payload());
    u_int64_t traj_timestamp = cloudToV.timestamp();
    std::cout << "timestamp: " << traj_timestamp << std::endl;
    auto C2V_planning = cloudToV.pathplanning();
    std::unique_ptr<std::vector<TrajectoryPose>> traj_from_cloud(new std::vector<TrajectoryPose>);
    for (cn::seisys::v2x::pb::PathPlanningPoint &point_tmp : C2V_planning)
    {
      TrajectoryPose pose_tmp;
      std::cout << "utmX: " << std::fixed << std::setprecision(3) << 1e-2 * point_tmp.pos().lat() << " m" << std::endl;
      std::cout << "utmY: " << std::fixed << std::setprecision(3) << 1e-2 * point_tmp.pos().lon() << " m" <<std::endl;
      std::cout << "heading: " << std::fixed << std::setprecision(3) << 0.0125 * point_tmp.heading() << " deg" << std::endl;
      std::cout << "speed: " <<  std::fixed << std::setprecision(3) << 0.02 * point_tmp.speed() << " m/s" << std::endl;
      std::cout << "relative time: " << std::fixed << std::setprecision(3) << 0.01 * point_tmp.estimatedtime() << " s" << std::endl;
      //std::cout << "acceleration: " << point_tmp.acceleration(). << std::endl;
      std::cout << "--------------------------------------------\n"<< std::endl;
      pose_tmp.x = 1e-2 * point_tmp.pos().lat();
      pose_tmp.y = 1e-2 * point_tmp.pos().lon();
      pose_tmp.theta = 0.0125 * M_PI / 180.0 * point_tmp.heading();
      pose_tmp.v = 0.02 * point_tmp.speed();
      pose_tmp.relative_time = 0.01 * point_tmp.estimatedtime();

      traj_from_cloud->push_back(pose_tmp);
    }
    std::cout << "Trajectory Size: " << traj_from_cloud->size() << std::endl;
    //std::cout << "GOT message:\n" << utme<< "\t" << utmn << std::endl;
    std::cout <<"--------------------------------------------\n"<< std::endl;
    //std::cout << "\tGot_Message: '" << vsmdata.DebugString()<< "'\n";
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

 public:
  callback(mqtt::async_client &cli, mqtt::connect_options &connOpts)
      : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}
};

int main(int argc, char *argv[])
{
  mqtt::connect_options connOpts;
  connOpts.set_keep_alive_interval(20);
  connOpts.set_clean_session(true);
  connOpts.set_password(PASSWORD);
  connOpts.set_user_name(USERNAME);
  mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

  callback cb(client, connOpts);
  client.set_callback(cb);

  try
  {
    std::cout << "Connecting to the MQTT server..." << std::flush;
    client.connect(connOpts, nullptr, cb);
  }
  catch (const mqtt::exception &)
  {
    std::cerr << "\nERROR: Unable to connect to MQTT server: '"
              << SERVER_ADDRESS << "'" << std::endl;
    return 1;
  }

  while (std::tolower(std::cin.get()) != 'q')
    ;

  try
  {
    std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
    client.disconnect()->wait();
    std::cout << "OK" << std::endl;
  }
  catch (const mqtt::exception &exc)
  {
    std::cerr << exc.what() << std::endl;
    return 1;
  }

  return 0;
}
