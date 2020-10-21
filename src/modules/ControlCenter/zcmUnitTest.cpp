/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-27 13:26:25
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-27 13:46:47
 */
#include <zcm/zcm-cpp.hpp>
#include <unistd.h>

#include "ControlCenterCommon.h"

#include "structCANCONTROL.hpp"
#include "structCANINFO.hpp"
#include "structREMOTECONTROL.hpp"

class messageHandle{
public:
    void veh_status_info(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO *msg){
        INFO("VEH_STATUS ==> car_speed: " << msg->carspeed << ", car_steer: " << msg->carsteer);
    }
};

int main(){

    zcm::ZCM veh_status_sub {""};
    zcm::ZCM veh_control_pub {""};
    
    if(!veh_status_sub.good()){
        ERR("veh_status_pub init error");
        return CC_ERR;
    }
    else if(!veh_control_pub.good()){
        ERR("veh_control_sub init error");
        return CC_ERR;
    }

    while(1){
        static int i = 0;
        i++;
        structCANCONTROL veh_control {};
        veh_control.aimspeed = i;
        veh_control.aimsteer = i*10;

        structREMOTECONTROL remote_control {};
        remote_control.enabled = i % 1;

        veh_control_pub.publish("REMOTECONTROL", &remote_control);
        veh_control_pub.publish("CANCONTROL", &veh_control);

        usleep(100*1000);
    }

    messageHandle msgHandle;
    veh_status_sub.subscribe("CANINFO", &messageHandle::veh_status_info, &msgHandle);

    veh_status_sub.run();

    return 0;
}
