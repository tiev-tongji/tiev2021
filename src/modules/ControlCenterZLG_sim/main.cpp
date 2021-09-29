/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName:
 * @Descripttion:
 * @Author: Ding Yongchao
 * @version:
 * @Date: 2019-10-25 20:40:07
 * @FunctionList:
 * @History:
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2020-11-04 21:37:46
  */
#include "ControlCenterCommon.h"
#include "EHBControl.hpp"
#include "ESRControl.hpp"
#include "ROEWECenterControl.h"
#include "messageControl.h"
#include "pidController.h"
#include "signal.h"
#include "structCANCONTROLZLG.hpp"
#include "zcm_receiver.h"
#include <iostream>
#include <math.h>
#include <unistd.h>

using namespace std;
double current_throttle;
double current_steer;
double current_brake;
int8_t current_reverse;

// void CANControlZLGThread() {
//   zcm::ZCM zcm{};
//   if (!zcm.good()) {
//     cout << "message publish zcm is not good" << endl;
//     return;
//   }

//   while (1) {
//     long start_time, end_time, used_time;

//     start_time = clock();

// // change zcmreceiver to messagecontrol
//     zcmreceiver.getCANControlInfo(current_timestamp, current_throttle,
//     current_steer, current_brake,current_reverse) {
//     structCANCONTROLZLG control_info;
//     struct timeval tv;
//     gettimeofday(&tv, NULL);
//     control_info.timestamp = tv.tv_sec * 1000000 + tv.tv_usec;
//     control_info.throttle = current_throttle;
//     control_info.steer = current_steer;
//     control_info.brake = current_brake;
//     control_info.reverse = current_reverse;
//     zcm.publish("CANCONTROLZLG", &control_info);
//     end_time = clock();
//     used_time =
//         1000 * (long)(1000.0 * (end_time - start_time) / CLOCKS_PER_SEC);
//     if (used_time < 20)
//       usleep((20 - used_time) * 1000); // freq:50Hz
//   }
// }

static control_params_t params;
const std::string params_file = "parameters.txt";
// ZcmReceiver zcmreceiver;

int main(int argc, char *argv[]) {
  // 参数载入
  load_params_file(params_file, &params);

  // registe ctrl-c
  // struct sigaction sigIntHandler;
  // sigIntHandler.sa_handler = exit_handler;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;
  // sigaction(SIGINT, &sigIntHandler, NULL);

  // ZCM消息中间件初始化
  messageControl msgControl;
  msgControl.init();

  bool is_break = false;
  float speed_torque = 0;
  float angle_torque = 0;
  float K = 0.0;
  // float pitch_max = 0;

  veh_info_t veh_info;            // htf : subcribe caninfo to get veh_info
  veh_info_t veh_pc_control_info; // htf : subscribe cancontrol info
  nav_info_t veh_nav_info;
  int prev_speed_sign = 0;
  zcm::ZCM zcm{};
  if (!zcm.good()) {
    cout << "message publish zcm is not good" << endl;
    return 0;
  }
  while (1) {
    // pitch_max = veh_nav_info.angle_pitch;
    long start_time, end_time, used_time;
    start_time = clock();

    msgControl.get_can_info_msg(&veh_info);
    msgControl.get_veh_control_msg(&veh_pc_control_info);
    msgControl.get_nav_info_msg(&veh_nav_info);
    msgControl.get_aim_path_msg(&K);

    ///////fix direction
    if (prev_speed_sign * veh_info.speed < -1e-1)
      veh_info.speed = -veh_info.speed;
    if (veh_info.speed > 1)
      prev_speed_sign = 1;
    else if (veh_info.speed < -1)
      prev_speed_sign = -1;
    else
      prev_speed_sign = 0;
    cout << "Actual Velocity:" << veh_info.speed
         << "Desired Velocity:" << veh_pc_control_info.speed << endl;

    // PID算法计算
    // speed_pid_control(veh_info.speed, veh_pc_control_info.speed, params,
    // &is_break, &speed_torque);
    // cout << "aim steer:" << veh_pc_control_info.angle << endl;
    speed_pid_control(veh_info.speed, veh_pc_control_info.speed,
                      veh_nav_info.angle_pitch, params, &is_break,
                      &speed_torque);
    angle_pid_control(veh_info, veh_pc_control_info.angle, K, params,
                      &angle_torque);

    speed_torque = fmax(fmin(speed_torque, 90.0), -80);
    angle_torque = fmax(fmin(angle_torque, 4.0), -4.0);

    // speed_torque = 90.0;
    double current_throttle = 0;
    double current_brake = 0;
    double current_steer = 0;
    int8_t current_reverse = 0;
    if (speed_torque >= 0) {
      current_throttle = speed_torque / 90;
      current_brake = 0;
    } else {
      current_throttle = 0;
      current_brake = fabs(speed_torque) / 80;
    }
    if (veh_pc_control_info.speed < 0)
      current_reverse = 1;
    else
      current_reverse = 0;
    current_steer = angle_torque / 4;
    cout << "ANGLE current steer:" << current_steer << endl;
    structCANCONTROLZLG control_info;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    control_info.timestamp = tv.tv_sec * 1000000 + tv.tv_usec;
    control_info.throttle = current_throttle;
    control_info.steer = current_steer;
    control_info.brake = current_brake;
    control_info.reverse = current_reverse;

    zcm.publish("CANCONTROLZLG", &control_info);
    end_time = clock();
    used_time =
        1000 * (long)(1000.0 * (end_time - start_time) / CLOCKS_PER_SEC);
    if (used_time < 20)
      usleep((20 - used_time) * 1000); // freq:50Hz
  }
  return 0;
}
