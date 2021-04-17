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
 * @LastEditTime: 2019-10-27 10:33:25
 */
#ifndef MESSAGE_CONTROL_H_
#define MESSAGE_CONTROL_H_

#include <iostream>
#include <mutex>
#include <zcm/zcm-cpp.hpp>
//#include <zcm/zcm.h>

#include "ControlCenterCommon.h"

#include "structCANCONTROL.hpp"
#include "structCANINFO.hpp"
#include "structREMOTECONTROL.hpp"
#include "structNAVINFO.hpp"
#include "structESROBJINFO.hpp"
#include "structAIMPATH.hpp"

class messageHandle{
public:
    STATE get_can_info_msg(veh_info_t* veh_info);
    STATE get_veh_control_msg(veh_info_t* control_info);
    STATE get_nav_info_msg(nav_info_t* nav_info);
    STATE get_veh_aimpath(float* curvature_);
public:
    void veh_can_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO *msg);
    void veh_control_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANCONTROL *msg);
    void veh_navinfo(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO *msg);
    void veh_aimpath(const zcm::ReceiveBuffer* rbuf,const std::string& chan,const structAIMPATH *msg);
    // TODO: acc使用pending
    //void veh_acc_info(const zcm_recv_buf_t *rbuf, const char *channel, const msg_t *msg, void *usr);
public:
    veh_info_t can_info_;
    veh_info_t control_info_;
    nav_info_t nav_info_;
    float curvature_;
    std::mutex control_info_lock;
    std::mutex can_info_lock;
    std::mutex nav_info_lock;
    std::mutex aim_path_lock;
};

class messageControl{
public:
    messageControl();
    ~messageControl();
public:
    STATE init();
    STATE get_veh_control_msg(veh_info_t* control_info);
    STATE get_can_info_msg(veh_info_t* veh_info);
    STATE get_nav_info_msg(nav_info_t* nav_info);
    //STATE pub_esr_map_msg(structESRMAP* esr_map);
    STATE get_aim_path_msg(float* curvature);
private:
    STATE nav_can_run();
    STATE can_control_run();
private:
    messageHandle msgHandle;
    // zcm::ZCM veh_status_pub {};
    zcm::ZCM nav_can_info_sub {};
    zcm::ZCM can_control_sub {};
    // zcm::ZCM veh_aim_path_sub {};
};

#endif // MESSAGE_CONTROL_H_
