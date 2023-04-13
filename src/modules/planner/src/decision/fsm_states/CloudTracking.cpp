#include <fstream>


#include "mqtt/client.h"
#include "mqtt/async_client.h"
#include "map_manager.h"
#include "tiev_fsm.h"
#include "tievlog.h"
#include "CloudSystem.h"
namespace TiEV {

using namespace std;


void CloudTracking::enter(Control& control) {
  entry_time = getTimeStamp();
  cout << "entry CloudTracking..." << endl;
}

void CloudTracking::update(FullControl& control) {
  if ((getTimeStamp() - entry_time) < 50e3) return;
  //LOG(INFO) << "Cloud Tracking update..." << endl;
  MapManager&     map_manager  = MapManager::getInstance();
  MessageManager& mesg_manager = MessageManager::getInstance();
  NavInfo cur_nav;
  if (mesg_manager.getNavInfo(cur_nav) == 0) {
    cerr << "mesg is unable to get navinfo" << endl;
  }
  auto cur_car = cur_nav.car_pose;
  /*
  
  std::cout <<" the car info is "<< std::endl;
  std::cout << (cur_car.utm_position.utm_x) << " " << cur_car.utm_position.utm_y
            << " " << cur_car.v << " " << (getTimeStamp() - entry_time) << endl;
  
  */

  static bool is_first_time = true;
  static mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
  if(is_first_time){
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    connOpts.set_password(PASSWORD);
    connOpts.set_user_name(USERNAME);
    callback cb(client, connOpts);
    client.set_callback(cb);
    try
    {
      std::cout << "Connecting to the MQTT server..." << std::flush;
      client.connect(connOpts, nullptr, cb)->wait();
      std::cout << "Connecting OK" << std::endl;
    }
    catch (const mqtt::exception &)
    {
      std::cerr << "\nERROR: Unable to connect to MQTT server: '"
                << SERVER_ADDRESS << "'" << std::endl;
      return;
    }
  }
  is_first_time = false;

  C2Vinfo cur_car_info = {cur_car.v, cur_car.utm_position.heading, cur_car.utm_position.utm_x,
                          cur_car.utm_position.utm_y};
  sendToCloud(cur_car_info,client);
  std::cout <<" send with car info " << cur_car.utm_position.utm_x <<"," << cur_car.utm_position.utm_y <<" heading: " << cur_car.utm_position.heading << std::endl;

  usleep(50 * 1e3);
}

}  // namespace TiEV
