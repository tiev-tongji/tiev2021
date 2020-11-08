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
    if(!veh_status_pub.good()){
        ERR("veh_status_pub init error");
        return CC_ERR;
    }
    else if(!veh_control_sub.good()){
        ERR("veh_control_sub init error");
        return CC_ERR;
    }

    veh_control_sub.subscribe("CANCONTROL", &messageHandle::veh_control_info, &msgHandle);
    veh_control_sub.subscribe("REMOTECONTROL", &messageHandle::veh_remote_control, &msgHandle);
    veh_control_sub.subscribe("NAVINFO", &messageHandle::veh_navinfo, &msgHandle);

    static std::thread zcmRun(&messageControl::zcm_run, this);

    return CC_OK;
}

STATE messageControl::zcm_run(){
    veh_control_sub.run();
    return CC_OK;
}

STATE messageControl::get_veh_control_msg(veh_info_t* veh_info){
    STATE ret = msgHandle.get_veh_control_msg(veh_info);
    return ret;
}

STATE messageControl::get_nav_info_msg(nav_info_t* nav_info){
    STATE ret = msgHandle.get_nav_info_msg(nav_info);
    return ret;
}

STATE messageControl::get_remote_control_msg(bool* remote_control){
    STATE ret = msgHandle.get_remote_control_msg(remote_control);
    return ret;
}

STATE messageControl::pub_veh_status_msg(veh_info_t& veh_info){
    struct timeval tv;
    gettimeofday(&tv, NULL);

    int steer = veh_info.angle;
    int speed = veh_info.speed * 100;
    // if (forwardOrBack == -1 && speed >0){
    //     speed = -speed;
    // }
    structCANINFO veh_status {};
    veh_status.carspeed = speed;
    veh_status.carsteer = steer;
    veh_status.timestamp = tv.tv_sec*1000000 + tv.tv_usec;
    
    veh_status_pub.publish("CANINFO", &veh_status);
}

STATE messageControl::pub_esr_objinfo_msg(structESROBJINFO* esrObjInfo){
    veh_status_pub.publish("ESROBJINFO", esrObjInfo);
}
STATE messageHandle::get_veh_control_msg(veh_info_t* veh_info){
    std::lock_guard<std::mutex> lk(veh_info_lock);
    *veh_info = veh_info_;
    return CC_OK;
}

STATE messageHandle::get_nav_info_msg(nav_info_t* nav_info){
    std::lock_guard<std::mutex> lk(nav_info_lock);
    *nav_info = nav_info_;
    return CC_OK;
}

STATE messageHandle::get_remote_control_msg(bool* remote_control){
    std::lock_guard<std::mutex> lk(remote_control_lock);
    *remote_control = remote_control_;
    return CC_OK;
}

void messageHandle::veh_control_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANCONTROL *msg){
    {
        std::lock_guard<std::mutex> lk(veh_info_lock);
        veh_info_.speed = msg->aimspeed;
        veh_info_.angle = msg->aimsteer;
    }
    INFO("ZCM INFO ==> Speed: " << veh_info_.speed << ", Angle: " << veh_info_.angle);
}

void messageHandle::veh_remote_control(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structREMOTECONTROL *msg){
    {
        std::lock_guard<std::mutex> lk(remote_control_lock);
        remote_control_ = msg->enabled;
    }
    INFO("ZCM INFO ==> remote_control: " << remote_control_);
}

void messageHandle::veh_navinfo(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO *msg){
    {
        std::lock_guard<std::mutex> lk(nav_info_lock);
        nav_info_.mHeading = msg->mHeading;
        nav_info_.angle_pitch = msg->mPitch + 2; //calibrated pitch angle on horizon surface
        nav_info_.utmX = msg->utmX;
        nav_info_.utmY = msg->utmY;
        nav_info_.speed = msg->mSpeed3d;
        nav_info_.yawRate = msg->mAngularRateZ;
        //nav_info_.pitchDeg = msg->mPitch;
    }
}
