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
 * @LastEditTime: 2019-11-09 21:37:46
 */
#include <iostream>
#include <unistd.h>

#include "ControlCenterCommon.h"
#include "ROEWECenterControl.h"
#include "pidController.h"
#include "messageControl.h"
#include "EHBControl.hpp"
#include "ESRControl.hpp"

static control_params_t params;
const std::string params_file = "parameters.txt";

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
    float pitch_max = 0;

    veh_info_t veh_info;
    while(1){
        // 获取ZCM发送过来的信息
        pitch_max = veh_nav_info.angle_pitch;
        msgControl.get_remote_control_msg(&enable_pc_control);
        msgControl.get_veh_control_msg(&veh_pc_control_info);
        msgControl.get_nav_info_msg(&veh_nav_info);
        INFO("enable_pc_control:" << (int)enable_pc_control); 
        if (veh_nav_info.angle_pitch > pitch_max)
        pitch_max = veh_nav_info.angle_pitch;
        std::cout << "angle_pitch:" << veh_nav_info.angle_pitch << "/tpitch_max:" << pitch_max << std::endl;
        //enable_pc_control = true;
        //veh_pc_control_info.speed = 0;
        //veh_pc_control_info.angle = 0;
        
        // 设置NAVINFO信息给ESR
        esr_control.setNavInfo(veh_nav_info); 
        // 获取ESR结果并发送
        esr_control.esrMapLock();
        msgControl.pub_esr_map_msg(esr_control.getEsrMapPtr());
        esr_control.esrMapUnLock();

        // 获取车身CAN信息
        veh_control.get_vehicle_info(&veh_info.speed, &veh_info.angle);
        INFO("================== Speed: " << veh_info.speed);
        msgControl.pub_veh_status_msg(veh_info);

        // PID算法计算
       // speed_pid_control(veh_info.speed, veh_pc_control_info.speed, params, &is_break, &speed_torque);
        speed_pid_control(veh_info.speed, veh_pc_control_info.speed, veh_nav_info.angle_pitch, params, &is_break, &speed_torque);
        angle_pid_control(veh_info, veh_pc_control_info.angle, params, &angle_torque);

        // 车身控制信号CAN发送
        // 刹车控制
        DCUMessage dcuMsg;
        //std::cout<< "isbreak: " << is_break << std:endl;
        //std::cout<< "speed_torque: " << speed_torque << std:endl;
        if(is_break == true){
            dcuMsg.AimPressure = speed_torque;
            speed_torque = 0;
        }
        // 油门控制
        else{
            dcuMsg.AimPressure = 1; //john: verify
        }
        ehb_control.sendDCUMessage(dcuMsg);
        if(!enable_pc_control){
            dcuMsg.AimPressure = 0;
        }
        veh_control.send_vehicle_control_info(speed_torque, angle_torque);
        veh_control.enable_vehicle_control(enable_pc_control);

        usleep(20 * 1000);
    }

    return 0;
}
