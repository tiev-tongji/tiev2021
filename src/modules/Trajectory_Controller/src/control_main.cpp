#include "controller_tools.h"
#include "stanley_tracking.h"
#include "structCANCONTROL.hpp"
#include "zcm_receiver.h"
#include "json/json.h"
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
#include <thread>
#include <time.h>
#include <unistd.h>

using namespace std;

ZcmReceiver zcmreceiver;
StanleyTracking stanleycontroller;
ControlPose ref_pose = {0, 0, 0, 0, 0};
ControlPose current_pose = {0, 0, 0, 0, 0};
vector<ControlPose> control_path;
double current_velocity;
double current_steering_angle;
double direction = 1; // forward:1 backward:-1
double current_yawrate;
double ref_velocity;
double ref_steering_angle;
double ref_velocity_filter = 0.0;
double ref_steering_angle_filter = 0.0;
double aim_dis_k;
double aim_dis_base_forward;
double aim_dis_base_backward;
double alfa_steering_angle;
double alfa_velocity;
double v_switch_high;
double ay_max_high;
double ay_max_low;
double max_wheel_angle;
double wheel_base;

void init() {
  ifstream ifs;
  ifs.open("../../../cfg/controllerPara.json");
  if (!ifs.is_open()) {
    cout << "can not open the controllerPara.json" << endl;
    return;
  }

  Json::Reader reader;
  Json::Value root;
  if (!reader.parse(ifs, root, false)) {
    cout << "can not parse the controllerPara.json" << endl;
    return;
  }

  double mass = root["vehiclePara"]["mass"].asDouble();
  double lf = root["vehiclePara"]["lf"].asDouble();
  double lr = root["vehiclePara"]["lr"].asDouble();
  double cornering_stiffness =
      root["vehiclePara"]["cornering_stiffness"].asDouble();
  wheel_base = root["vehiclePara"]["wheel_base"].asDouble();
  max_wheel_angle = root["vehiclePara"]["max_wheel_angle"].asDouble();

  double k_lateral_front = root["controlPara"]["k_lateral_front"].asDouble();
  double k_lateral_rear = root["controlPara"]["k_lateral_rear"].asDouble();
  double k_heading_dev = root["controlPara"]["k_heading_dev"].asDouble();
  double k_lateral_front_high =
      root["controlPara"]["k_lateral_front_high"].asDouble();
  double k_heading_dev_high =
      root["controlPara"]["k_heading_dev_high"].asDouble();
  double flag_dynamic = root["controlPara"]["flag_dynamic"].asDouble();
  double k_kappa_dev = root["controlPara"]["k_kappa_dev"].asDouble();
  v_switch_high = root["controlPara"]["v_switch_high"].asDouble();
  ay_max_high = root["controlPara"]["ay_max_high"].asDouble();
  ay_max_low = root["controlPara"]["ay_max_low"].asDouble();
  aim_dis_k = root["controlPara"]["aim_dis_k"].asDouble();
  aim_dis_base_forward = root["controlPara"]["aim_dis_base_forward"].asDouble();
  aim_dis_base_backward =
      root["controlPara"]["aim_dis_base_backward"].asDouble();

  alfa_steering_angle = root["filterPara"]["alfa_steering_angle"].asDouble();
  alfa_velocity = root["filterPara"]["alfa_velocity"].asDouble();

  stanleycontroller.setVehicleParameter(wheel_base, mass, lf, lr,
                                        cornering_stiffness, max_wheel_angle);
  stanleycontroller.setControlParameter(
      k_lateral_front, k_lateral_rear, k_heading_dev, k_lateral_front_high,
      k_heading_dev_high, flag_dynamic, k_kappa_dev);

  cout << "wheel_base: " << wheel_base << endl;
  cout << "mass: " << mass << endl;
  cout << "lf: " << lf << endl;
  cout << "lr: " << lr << endl;
  cout << "cornering_stiffness: " << cornering_stiffness << endl;
  cout << "max_wheel_angle: " << max_wheel_angle << endl;

  cout << "k_lateral_front: " << k_lateral_front << endl;
  cout << "k_lateral_rear: " << k_lateral_rear << endl;
  cout << "k_heading_dev: " << k_heading_dev << endl;
  cout << "k_lateral_front_high: " << k_lateral_front_high << endl;
  cout << "k_heading_dev_high: " << k_heading_dev_high << endl;
  cout << "flag_dynamic: " << flag_dynamic << endl;
  cout << "k_kappa_dev: " << k_kappa_dev << endl;

  cout << "alfa_steering_angle: " << alfa_steering_angle << endl;
  cout << "alfa_velocity: " << alfa_velocity << endl;
  cout << "aim_dis_k: " << aim_dis_k << endl;
  cout << "aim_dis_base_forward: " << aim_dis_base_forward << endl;
  cout << "aim_dis_base_backward: " << aim_dis_base_backward << endl;
}

void StanleyContollerThread() {
  zcm::ZCM zcm{};
  if (!zcm.good()) {
    cout << "message publish zcm is not good" << endl;
    return;
  }

  while (1) {
    long start_time, end_time, used_time;

    start_time = clock();

    zcmreceiver.getCarStatus(current_velocity, current_steering_angle,
                             current_yawrate);
    //		current_velocity = 60; //add for test
    double current_velocity_m_s = current_velocity / 3.6;
    cout << "current vel: " << current_velocity_m_s << endl;
    if (zcmreceiver.getControlPath(control_path)) {
      double aim_dis_base = aim_dis_base_forward;
      if (direction == -1) {
        aim_dis_base = aim_dis_base_backward;
      }
      //			getRefPose3(control_path, direction,
      // current_pose, ref_pose);
      //			getRefPose(control_path, direction,
      // current_pose, ref_pose);
      getRefPose2(control_path, current_velocity_m_s, aim_dis_k, aim_dis_base,
                  ref_pose);
      ref_velocity = getRefVelocity(control_path, current_velocity_m_s,
                                    aim_dis_k, aim_dis_base);
      if (ref_velocity < 0)
        direction = -1;
      else
        direction = 1;

    } else {
      ref_velocity = 0.0;
    }
    // cout << "ref pose: " <<  ref_pose.x << "  " << ref_pose.y << "  " <<
    // ref_pose.angle * 180 / PI << "  " << ref_pose.kappa << endl;

    double ay_max;
    if (current_velocity < v_switch_high) {
      stanleycontroller.computeWheelAngle(ref_pose, current_pose,
                                          current_velocity_m_s, direction,
                                          current_yawrate, ref_steering_angle);
      ay_max = ay_max_low;
    } else {
      stanleycontroller.computeWheelAngleHighSpeed(
          ref_pose, current_pose, current_velocity_m_s, direction,
          current_yawrate, ref_steering_angle);
      ay_max = ay_max_high;
    }
    double soft = 0.1;
    double wheel_angle_max =
        min(atan(wheel_base * ay_max * 9.8 /
                 (current_velocity_m_s * current_velocity_m_s + soft)),
            max_wheel_angle);
    ref_steering_angle_filter = filter(
        ref_steering_angle, ref_steering_angle_filter, alfa_steering_angle);
    ref_velocity_filter =
        filter(ref_velocity, ref_velocity_filter, alfa_velocity);

    end_time = clock();
    used_time =
        1000 * (long)(1000.0 * (end_time - start_time) / CLOCKS_PER_SEC);
    if (used_time < 20)
      usleep((20 - used_time) * 1000); // freq:50Hz

    structCANCONTROL control_msg;
    double steer_direction = ref_steering_angle_filter > 0 ? 1 : -1;
    cout << "steer_direction--------------------------------> "
         << steer_direction << endl;
    cout << "wheel_angle_max--------------------------------> "
         << wheel_angle_max << endl;

    control_msg.aimsteer =
        steer_direction *
        min(fabs(ref_steering_angle_filter), wheel_angle_max) * (180.0 / PI) *
        17.4; // rad to deg, add trans ratio
    control_msg.aimspeed = ref_velocity_filter * 3.6;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    control_msg.timestamp = tv.tv_sec * 1000000 + tv.tv_usec;

    cout << "aimsteer---------------------------------------> "
         << control_msg.aimsteer << endl;
    cout << "aimspeed---------------------------------------> "
         << control_msg.aimspeed << endl;

    zcm.publish("CANCONTROL", &control_msg);
  }
}

int main(int argc, char *argv[]) {
  init();
  thread task1(StanleyContollerThread);
  task1.detach();

  zcmreceiver.zcmMsgReceive();
  return 0;
}
