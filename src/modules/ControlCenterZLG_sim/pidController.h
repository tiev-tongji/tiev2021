/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 22:34:10
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-25 23:15:38
 */
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include "ControlCenterCommon.h"

//STATE speed_pid_control(const float& veh_speed, float& desired_speed, const control_params_t& params, bool* is_break, float* control_output);
STATE speed_pid_control(const float& veh_speed, float& desired_speed, float& angle_pitch, const control_params_t& params, bool* is_break, float* control_output);

STATE angle_pid_control(const veh_info_t& veh_info, float& desired_angle, float R , const control_params_t& params, float* control_output);

#endif // PID_CONTROLLER_H_
