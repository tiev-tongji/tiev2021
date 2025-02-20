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
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "ControlCenterCommon.h"
#include "ROEWECenterControl.h"
#include "pidController.h"
#include "messageControl.h"
#include "EHBControl.hpp"
#include "ESRControl.hpp"

static control_params_t params;
const std::string params_file = "parameters.txt";
// static float min_speed_torque = 1000;

int main(){
    // 参数载入
    load_params_file(params_file, &params);
    
    // ZCM消息中间件初始化
    messageControl msgControl;
    msgControl.init();

    // 车身CAN控制初始化
    ROEWEControl veh_control;
    veh_control.init();
    
    EHBControl ehb_control;
    ehb_control.init();
    
    //for esr
    ESRControl esr_control(ehb_control.getCANPort());
    esr_control.init();
    nav_info_t veh_nav_info;
    
    bool enable_pc_control = false;
    veh_info_t veh_pc_control_info;

    bool is_break = false;
    float speed_torque = 0;
    float angle_torque = 0;
    float K = 0.0;
    // float pitch_max = 0;
    // float pitch_min = 0;

    veh_info_t veh_info;
    int prev_speed_sign = 0;
    while(1){
        // 获取ZCM发送过来的信息
        // pitch_max = veh_nav_info.angle_pitch;
        msgControl.get_remote_control_msg(&enable_pc_control);
        msgControl.get_veh_control_msg(&veh_pc_control_info);
        msgControl.get_nav_info_msg(&veh_nav_info);
        INFO("enable_pc_control:" << (int)enable_pc_control); 
        msgControl.get_aim_path_msg(&K);
       // if (veh_nav_info.angle_pitch > pitch_max)
	//	pitch_max = veh_nav_info.angle_pitch;
	// if (veh_nav_info.angle_pitch < pitch_min)
	// 	pitch_min = veh_nav_info.angle_pitch;
    //     std::cout << "angle_pitch:" << veh_nav_info.angle_pitch << "\npitch_max:" << pitch_max << "\npitch_min:" << pitch_min << std::endl;
        //enable_pc_control = true;
        //veh_pc_control_info.speed = 0;
        //veh_pc_control_info.angle = 0;
        
        // 设置NAVINFO信息给ESR
        esr_control.setNavInfo(veh_nav_info); 

        // 获取ESR结果并发送
        esr_control.esrObjInfoLock();
        msgControl.pub_esr_objinfo_msg(esr_control.getEsrObjInfoPtr());
        esr_control.esrObjInfoUnLock();

        // 获取车身CAN信息
        veh_control.get_vehicle_info(&veh_info.speed, &veh_info.angle);
        INFO("================== Speed: " << veh_info.speed);
         ///////fix direction
    if (prev_speed_sign * veh_info.speed < -1e-1)
      veh_info.speed = -veh_info.speed;
    if (veh_info.speed > 1)
      prev_speed_sign = 1;
    else if (veh_info.speed < -1)
      prev_speed_sign = -1;
    else
      prev_speed_sign = 0;
        msgControl.pub_veh_status_msg(veh_info);

        // PID算法计算
        speed_pid_control(veh_info.speed, veh_pc_control_info.speed,
                      veh_nav_info.angle_pitch, params, &is_break,
                      &speed_torque);
    angle_pid_control(veh_info, veh_pc_control_info.angle, K, params,
                      &angle_torque);
        // 车身控制信号CAN发送
        // 刹车控制
        DCUMessage dcuMsg;
         std::cout<< "isbreak: " << is_break << std::endl;
         std::cout<< "speed_torque: " << speed_torque << std::endl;
// if(speed_torque <= min_speed_torque) min_speed_torque = speed_torque;
//         std::cout<< "min_speed_torque: " << min_speed_torque << std::endl;
        if(is_break == true && enable_pc_control){
            dcuMsg.AimPressure = fmin(80.0, fabs(speed_torque));
            speed_torque = 0;
        }
        // 油门控制
        else{
            dcuMsg.AimPressure = 0; //john: verify
	}
	if(dcuMsg.AimPressure > 0)
		std::cout<< "aimpressure " << int(dcuMsg.AimPressure)<< std::endl;
        if(!enable_pc_control){
            dcuMsg.AimPressure = 0;
        }
	ehb_control.sendDCUMessage(dcuMsg);
        veh_control.enable_vehicle_control(enable_pc_control);
        
    // TODO ChenKai Review
    speed_torque = fmin(speed_torque, 90.0);
    angle_torque = fmax(fmin(angle_torque, 4.0), -4.0);
    // speed_torque = 90.0;
    // double current_throttle = 0;
    // double current_brake = 0;
    // double current_steer = 0;
    // int8_t reverse = 0;
    // if (speed_torque >= 0) {
    //   current_throttle = current_speed_torque / 90;
    //   current_brake = 0;
    // } else {
    //   current_throttle = 0;
    //   current_brake = fabs(speedtorque) / 80;
    // }
    // if (veh_pc_control_info.speed < 0)
    //   current_reverse = 1;
    // else
    //   current_reverse = 0;
    // current_steer = angle_torque / 4;
        veh_control.send_vehicle_control_info(speed_torque, angle_torque);

        usleep(20 * 1000);
    }

    return 0;
}
