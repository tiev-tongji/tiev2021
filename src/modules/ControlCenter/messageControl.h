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
#include "structESRMAP.hpp"

class messageHandle{
public:
    STATE get_veh_control_msg(veh_info_t* veh_info);
    STATE get_remote_control_msg(bool* remote_control);
    STATE get_nav_info_msg(nav_info_t* nav_info);
public:
    void veh_control_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANCONTROL *msg);
    void veh_remote_control(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structREMOTECONTROL *msg);
    void veh_navinfo(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO *msg);
    // TODO: acc使用pending
    //void veh_acc_info(const zcm_recv_buf_t *rbuf, const char *channel, const msg_t *msg, void *usr);
public:
    veh_info_t veh_info_;
    nav_info_t nav_info_;
    bool remote_control_ = false;
    std::mutex veh_info_lock;
    std::mutex remote_control_lock;
    std::mutex nav_info_lock;
};

class messageControl{
public:
    messageControl();
    ~messageControl();
public:
    STATE init();
    STATE get_veh_control_msg(veh_info_t* veh_info);
    STATE get_remote_control_msg(bool* remote_control);
    STATE pub_veh_status_msg(veh_info_t& veh_info);
    STATE get_nav_info_msg(nav_info_t* nav_info);
    STATE pub_esr_map_msg(structESRMAP* esr_map);
private:
    STATE zcm_run();
private:
    messageHandle msgHandle;
    zcm::ZCM veh_status_pub {};
    zcm::ZCM veh_control_sub {};
};

#endif // MESSAGE_CONTROL_H_
