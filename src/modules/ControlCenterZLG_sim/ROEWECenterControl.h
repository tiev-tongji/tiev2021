/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 19:08:45
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-27 14:01:20
 * @LastEditors: Junqiao Zhao 
 * @LastEditTime: 2020-11-14 14:01:20
 */
#ifndef ROEWE_CENTER_CONTROL_H_
#define ROEWE_CENTER_CONTROL_H_

#include <iostream>
#include <linux/can.h>
#include <atomic>
#include "zlg_can.h"
#include "ROEWECANDefine.h"
#include "ControlCenterCommon.h"

class ROEWEControl{
public:
    ROEWEControl();
    ~ROEWEControl();
public:
    STATE init();
    STATE get_vehicle_info(float* car_speed, float* car_angle);
    STATE send_vehicle_control_info(float& speed_torque, float& angle_torque);
    STATE enable_vehicle_control(bool enable_control);
private:
    STATE can_socket_open();
    // 线程函数，用于起独立线程后，接收与发送CAN消息
    void get_can_info();
    void send_can_info();
private:
    // 方向盘转向获取
    STATE get_m_EPS_HSC_FrP01(VCI_CAN_OBJ *frame, float* car_angle);
    // 车速获取
    STATE get_m_VCU2MAB_2(VCI_CAN_OBJ * frame, float* car_speed);
    // 档位启停控制信号获取
    STATE get_m_EPS2VMS(VCI_CAN_OBJ * frame, unsigned char* control_mode);
    // TODO: 暂时无用
    STATE get_m_VCU2MAB_3(VCI_CAN_OBJ * frame) {return CC_OK;}
    STATE get_m_VCU2MAB_4(VCI_CAN_OBJ * frame) {return CC_OK;}
    STATE get_m_VCU2MAB_5(VCI_CAN_OBJ * frame) {return CC_OK;}
    STATE get_m_BSC2MCU(VCI_CAN_OBJ * frame) {return CC_OK;}

    STATE send_m_VMS2EPS(VCI_CAN_OBJ * frame, float& angle_torque, unsigned char enable_control = 1);
    STATE send_m_MAB2VCU(VCI_CAN_OBJ * frame, float& speed_torque, unsigned char enable_control = 1);
private:
    int can_fd = 0;
    // 车辆实际返回
    std::atomic<float> car_angle_;
    std::atomic<float> car_speed_;
    // 控制期望输出
    std::atomic<float> speed_torque_;
    std::atomic<float> angle_torque_;
    // 车辆受控标志位
    std::atomic<bool> enable_control_;
    std::atomic<uint8_t> veh_control_mode_;
public:
    CAN_DEV_INFO can_dev;
    VCI_INIT_CONFIG config;
    unsigned int rcv_wait_time;
    unsigned int rcv_buff_size;
};

#endif // ROEWE_CENTER_CONTROL_H_
