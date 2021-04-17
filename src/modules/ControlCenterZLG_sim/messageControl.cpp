/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-27 09:51:15
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-27 13:44:04
 */
#include <thread>
#include "messageControl.h"
    
messageControl::messageControl(){
    
}

messageControl::~messageControl(){

}

STATE messageControl::init(){
    // if(!veh_status_pub.good()){
    //     ERR("veh_status_pub init error");
    //     return CC_ERR;
    // }
    if(!nav_can_info_sub.good()){
        ERR("can_info_sub init error");
        return CC_ERR;
    }
    if(!can_control_sub.good()){
        ERR("can_control_sub init error");
        return CC_ERR;
    }

    can_control_sub.subscribe("CANCONTROL", &messageHandle::veh_control_info, &msgHandle);
    nav_can_info_sub.subscribe("CANINFO", &messageHandle::veh_can_info, &msgHandle);
    nav_can_info_sub.subscribe("NAVINFO", &messageHandle::veh_navinfo, &msgHandle);
    can_control_sub.subscribe("AIMPATH", &messageHandle::veh_aimpath, &msgHandle);

    static std::thread navcanRun(&messageControl::nav_can_run, this);
    static std::thread cancontrolRun(&messageControl::can_control_run, this);

    return CC_OK;
}

STATE messageControl::nav_can_run(){
    nav_can_info_sub.run();
    return CC_OK;
}

STATE messageControl::can_control_run(){
    can_control_sub.run();
    return CC_OK;
}

STATE messageControl::get_veh_control_msg(veh_info_t* control_info){
    STATE ret = msgHandle.get_veh_control_msg(control_info);
    return ret;
}

STATE messageControl::get_nav_info_msg(nav_info_t* nav_info){
    STATE ret = msgHandle.get_nav_info_msg(nav_info);
    return ret;
}

STATE messageControl::get_can_info_msg(veh_info_t* can_info){
    STATE ret = msgHandle.get_can_info_msg(can_info);
    return ret;
}

STATE messageControl::get_aim_path_msg(float* curvature){
    STATE ret = msgHandle.get_veh_aimpath(curvature);
    return ret;
}


STATE messageHandle::get_veh_control_msg(veh_info_t* control_info){
    std::lock_guard<std::mutex> lk(control_info_lock);
    *control_info = control_info_;
    return CC_OK;
}

STATE messageHandle::get_nav_info_msg(nav_info_t* nav_info){
    std::lock_guard<std::mutex> lk(nav_info_lock);
    *nav_info = nav_info_;
    return CC_OK;
}

STATE messageHandle::get_can_info_msg(veh_info_t* can_info){
    std::lock_guard<std::mutex> lk(can_info_lock);
    *can_info = can_info_;
    return CC_OK;
}

STATE messageHandle::get_veh_aimpath(float* curvature){
    std::lock_guard<std::mutex> lk(aim_path_lock);
    *curvature = curvature_;
    return CC_OK;
}

void messageHandle::veh_control_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANCONTROL *msg){
    {
        std::lock_guard<std::mutex> lk(control_info_lock);
        control_info_.speed = msg->aimspeed;
        control_info_.angle = msg->aimsteer;
    }
    INFO("ZCM INFO ==> control info: " << "speed: " << msg->aimspeed << ", Angle: " << msg->aimsteer);
}

void messageHandle::veh_can_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO *msg){
    {
        std::lock_guard<std::mutex> lk(can_info_lock);
        can_info_.speed = msg->carspeed;
        can_info_.angle = msg->carsteer;
    }
    INFO("ZCM INFO ==> can_info: " << "Speed: " << msg->carspeed <<", Angle: "<< msg->carsteer);
}

void messageHandle::veh_navinfo(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO *msg){
    {
        std::lock_guard<std::mutex> lk(nav_info_lock);
        nav_info_.mHeading = msg->mHeading;
        nav_info_.angle_pitch = msg->mPitch+2.96;
        nav_info_.utmX = msg->utmX;
        nav_info_.utmY = msg->utmY;
        nav_info_.speed = msg->mSpeed3d;
        nav_info_.yawRate = msg->mAngularRateZ;
        //nav_info_.pitchDeg = msg->mPitch;
    }
    INFO("ZCM INFO ==> nav_info");
}
void messageHandle::veh_aimpath(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structAIMPATH *msg){
    {
        std::lock_guard<std::mutex> lk(aim_path_lock);
        if(msg->points[0].k < 0.2)
            curvature_ = msg->points[0].k;
        //nav_info_.pitchDeg = msg->mPitch;
    }
    INFO("ZCM INFO ==> veh aimpath");
}
