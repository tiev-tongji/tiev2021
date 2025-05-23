/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 22:33:30
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-25 23:41:06
 */
#include "pidController.h"
#include <math.h>
#define PID_DEBUG 1

const int veh_stop_speed = 1;
const int veh_break_enable = 3;
const int veh_limit_speed_change = 10;
const int veh_high_speed = 40;
const int veh_limit_angle = 40;
const int veh_limit_angle_change = 20;
const float TOL = 0.1;
const float L = 2.3;
const float L_r = 1.2623;
const float m_f = 601.5;
const float m_r = 494.5;
const float C_alpha_f = 60164;
const float C_alpha_r = 63776;

// 原先的期望速度
static float old_desired_speed = 0;
// 速度微分项需要使用参数
static float P_speed_old = 0;
// 速度积分项
static float I_speed = 0;

// 角度微分项需要使用参数
static float P_angle_old = 0;
// 角度积分项
static float I_angle = 0;

static float I_speed_limit = 600;  // 2*50hz*3s
static float I_angle_limit = 3000; // 2*50hz*3s
float Kv = m_f / (2 * C_alpha_f) - m_r / (2 * C_alpha_r);

// 默认正负一致
inline void clamp(float &input, const float &hi)
{
    if (input < -(hi))
    {
        input = -(hi);
    }
    else if (input > hi)
    {
        input = hi;
    }
    return;
}

//STATE speed_pid_control(const float& veh_speed, float& desired_speed, const control_params_t& params, bool* is_break, float* control_output){
STATE speed_pid_control(const float &veh_speed, float &desired_speed, float &angle_pitch, const control_params_t &params, bool *is_break, float *control_output)
{
    float FF_valve = 2;
    float FF_valve_throttle = 2;
    if (PID_DEBUG)
        INFO("acc_P: " << params.acc_P << "desired_speed: " << desired_speed);
    float F_t = 0;     // Real_time output Torque
    float P_speed = 0; // Real_time Error speed

    // // 原有速度与现有速度的期望反向时，先进行停车处理
    // // TODO: 考虑更新过快时候的问题
    // if(old_desired_speed * desired_speed < 0 && veh_speed != 0){
    // 	desired_speed = 0;
    // }

    //// 期望速度与实际速度反向时，先进行停车处理
    //if(desired_speed * veh_speed < 0){
    //    desired_speed = 0;
    //}

    int8_t enable_forward = 1; // 默认向前开车

    // 预设速度小于0，并且实际速度低于一定值，则可进行倒车
    if (desired_speed < 0 && veh_speed <= veh_stop_speed)
    {
        enable_forward = -1;
    }

    // TODO: 确认是否存在负的速度，需要在外面进行处理，看是否能处理
    // Car_Speed *= forwardOrBack;

    P_speed = desired_speed - veh_speed;
    old_desired_speed = desired_speed;

    // 对P值进行最大的限定
    clamp(P_speed, veh_limit_speed_change);

    // 有一定速度后，才开始使用积分
    if (fabs(veh_speed) == 0)
    {
        if (I_speed >= I_speed_limit)
            I_speed = 0;
        else
            I_speed = I_speed + P_speed;
    }
    else
    {
        if (fabs(P_speed) > 1)
            I_speed = I_speed + P_speed;
        else
            I_speed = 0;
    }
    // 50HZ
    clamp(I_speed, I_speed_limit);
    //INFO("I_Speed: " << I_speed);

    // 微分项处理
    float D_speed = 0;
    D_speed = P_speed - P_speed_old;
    P_speed_old = P_speed;

    // 速度为正时，期望速度大于实际速度，加速
    // 速度为负时，期望速度小于实际速度，后退加速
    // TODO: 速度超过一点点如何处理？
    // INFO("------------------------------------------P_speed:" << P_speed << " veh_speed:" << veh_speed << " veh_break_enable:" << veh_break_enable);
    if (P_speed * veh_speed >= 0 || -P_speed < veh_break_enable)
    {
        *is_break = false;
        float P_contribute = P_speed * params.acc_P;
        float I_contribute = I_speed * params.acc_I;
        float D_contribute = D_speed * params.acc_D;
        float FF_contribute_throttle = 0;
        if (angle_pitch > FF_valve_throttle)
            FF_contribute_throttle = angle_pitch * (params.acc_FF);
        else if (angle_pitch < -FF_valve_throttle)
        {
            FF_contribute_throttle = angle_pitch * (params.break_FF);
        }
        *control_output = FF_contribute_throttle + P_contribute + I_contribute + D_contribute;
        if (PID_DEBUG)
            INFO("ACC INFO ==> P: " << P_contribute << ", "
                                    << "I: " << I_contribute << ", "
                                    << "D: " << D_contribute << ","
                                    << "FF:" << FF_contribute_throttle);
    }
    // 需要进行减速处理
    else
    {
        *is_break = true;
        float P_contribute = P_speed * params.break_P;
        float I_contribute = I_speed * params.break_I;
        float D_contribute = D_speed * params.break_D;
        //*control_output = -(P_contribute + I_contribute + D_contribute);
        float feedfoward_contribute = 0;
        if (angle_pitch > FF_valve_throttle)
            feedfoward_contribute = -angle_pitch * (params.acc_FF);
        else if (angle_pitch < -FF_valve_throttle)
        {
            feedfoward_contribute = -angle_pitch * (params.break_FF);
        }
        *control_output = -(feedfoward_contribute + P_contribute + I_contribute + D_contribute);

        //INFO("BREAK INFO ==> P: " << P_contribute << ", " << "I: " << I_contribute << ", " << "D: " << D_contribute);
        if (PID_DEBUG)
            INFO("BREAK INFO ==> P: " << P_contribute << ", "
                                      << "I: " << I_contribute << ", "
                                      << "D: " << D_contribute << ", "
                                      << "FF: " << feedfoward_contribute);
    }

    return CC_OK;
}

STATE angle_pid_control(const veh_info_t &veh_info, float &desired_angle, float Cur, const control_params_t &params, float *control_output)
{

    // 正常模式计算
    float k = 1;
    float P_angle = desired_angle - veh_info.angle;
    if (fabs(P_angle) > params.steer_valve)
        k = params.steer_change_coe;

    if (veh_info.speed >= 40)
    {
        // 速度较高时，最大转角限制
        clamp(desired_angle, veh_limit_angle);

        P_angle = desired_angle - veh_info.angle;

        // 最大角度变化限制
        clamp(P_angle, veh_limit_angle_change);
    }

    // 几乎处于停止状态时，不进行积分
    if (veh_info.speed < veh_stop_speed || fabs(P_angle) < 20)
    {
        I_angle = 0;
    }
    else
    {
        // 积分处理
        I_angle += P_angle;
    }
    clamp(I_angle, I_angle_limit);

    // 微分项处理
    float D_angle = P_angle - P_angle_old;
    P_angle_old = P_angle;
    float P_contribute, I_contribute, D_contribute, FF_contribute = 0;
    if (veh_info.speed <= 20)
    {
        P_contribute = P_angle * params.steer_P * k;
        I_contribute = I_angle * params.steer_I;
        D_contribute = D_angle * params.steer_D;
        //DEBUG("Curvature INFO ==> curvature:" << Cur);
        FF_contribute = L * Cur + Kv * veh_info.speed * veh_info.speed * Cur - params.steer_FF * (-L_r * Cur + m_r / (2 * C_alpha_r) * veh_info.speed * veh_info.speed * Cur);
    }
    else
    {
        P_contribute = P_angle * params.steer_PH * k;
        I_contribute = I_angle * params.steer_IH;
        D_contribute = D_angle * params.steer_DH;

        //DEBUG("Curvature INFO ==> curvature:" << Cur);
        FF_contribute = L * Cur + Kv * veh_info.speed * veh_info.speed * Cur - params.steer_FFH * (-L_r * Cur + m_r / (2 * C_alpha_r) * veh_info.speed * veh_info.speed * Cur);
    }

    if (PID_DEBUG)
    {
        DEBUG("ANGLE INFO ==> P: " << P_contribute << ", "
                                   << "I: " << I_contribute << ", "
                                   << "D: " << D_contribute << " FF: " << FF_contribute << " KV:  " << Kv);
        DEBUG("ANGLE INFO ==> desired_angle:" << desired_angle << " veh_ang:" << veh_info.angle << " P_angle:" << P_angle);
    }

    *control_output = P_contribute + I_contribute + D_contribute + FF_contribute;

    return CC_OK;
}
