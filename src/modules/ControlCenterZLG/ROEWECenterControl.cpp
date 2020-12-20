/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 19:08:31
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-27 14:01:29
 * @LastEditors: Junqiao Zhao 
 * @LastEditTime: 2020-11-14 14:01:20
 */
#include <thread>
#include <unistd.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include "ROEWECenterControl.h"


ROEWEControl::ROEWEControl(){
	car_angle_ = 0;
	car_speed_ = 0;
	speed_torque_ = 0;
	angle_torque_ = 0;
	enable_control_ = false;
	
}

ROEWEControl::~ROEWEControl(){

}

STATE ROEWEControl::init(){
    rcv_buff_size = 1000;
    rcv_wait_time = 100;
    // 启动两个线程，分别为CAN消息接收线程与CAN消息发送线程
    INFO("Start to subscribe CAN0 info");
    static std::thread get_info(&ROEWEControl::get_can_info, this);
	INFO("\n\n\nstart get_can_info thread\n\n\n");
    //get_info.join();
    usleep(100*1000);
    
    INFO("Start to publish CAN0 info");
    static std::thread send_info(&ROEWEControl::send_can_info, this);
    //send_info.join();
    usleep(100*1000);

    return CC_OK;
}

void ROEWEControl::get_can_info(){
    int nbytes;
    float car_angle, car_speed;
    unsigned char veh_control_mode;
    VCI_CAN_OBJ can[rcv_buff_size];
    while(1){
	    uint32_t cnt = VCI_Receive(can_dev.devType, can_dev.devIndex, 
			    can_dev.channelNum, can, rcv_buff_size, rcv_wait_time);
	    //printf("\ncnt = %d\n", cnt);
	    for(int i = 0; i < cnt; i++){
		    switch(can[i].ID){
			    case 0x18B:
				    this->get_m_EPS_HSC_FrP01(&can[i], &car_angle);
				    car_angle_ = car_angle;
				    DEBUG("CANINFO: car_angle ==> " << car_angle_);
				    break;
			    case 0x302:
				    this->get_m_VCU2MAB_2(&can[i], &car_speed);
				    car_speed_ = car_speed;
				    DEBUG("CANINFO: car_speed ==> " << car_speed_);
				    break;
			    case 0x312:
				    this->get_m_EPS2VMS(&can[i], &veh_control_mode);
				    veh_control_mode_ = veh_control_mode;
				    DEBUG("CANINFO: veh_control_mode ==> " << int(veh_control_mode_));
				    break;

			    default:
				    break;
		    }
	    }
	    // 100HZ 接收
	    usleep(10*1000);
    }
}

void ROEWEControl::send_can_info(){
	int nbytes;
	float angle_torque, speed_torque;
	VCI_CAN_OBJ frame[3];
	VCI_CAN_OBJ canObj[2];

	while(true){
		// 100HZ 发送
		usleep(10*1000);
		// 无PC授权控制信号
		if(1 != enable_control_){
			continue;
		}
		// 无车辆授权控制信号
		//if(veh_control_mode_ == 0){
		//	INFO("Waitting for Gear Operation ......");
		//	continue;
		//}
		angle_torque = angle_torque_;
		speed_torque = speed_torque_;
		this->send_m_VMS2EPS(&canObj[0], angle_torque);
		this->send_m_MAB2VCU(&canObj[1], speed_torque);
		DEBUG("CAN_CONTROL_INFO: speed_torque ==> " << speed_torque);
		DEBUG("CAN_CONTROL_INFO: angle_torque ==> " << angle_torque);
		 /* Transmit the can message */
        VCI_Transmit(can_dev.devType, can_dev.devIndex, 
                        can_dev.channelNum, &canObj[0], 1);
        VCI_Transmit(can_dev.devType, can_dev.devIndex, 
                        can_dev.channelNum, &canObj[1], 1);
		// if(nbytes != sizeof(frame)){
		// 	ERR("CAN0 Send ERROR!");
		// }
	}
}

// // 默认CAN打开CAN0口进行接收与控制
// STATE ROEWEControl::can_socket_open(){
// 	struct sockaddr_can addr_can0;
// 	struct ifreq ifr_can0;
// 	struct can_filter rfilter_can0[3];
// 	can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     strcpy(ifr_can0.ifr_name, "can0");
// 	ioctl(can_fd, SIOCGIFINDEX, &ifr_can0);
// 	addr_can0.can_family = AF_CAN;
// 	addr_can0.can_ifindex = ifr_can0.ifr_ifindex;
	
// 	bind(can_fd, (struct sockaddr *)&addr_can0, sizeof(addr_can0));
	
// 	rfilter_can0[0].can_id = 0x18B;
//     rfilter_can0[0].can_mask = CAN_SFF_MASK;
// 	rfilter_can0[1].can_id = 0x302;
//     rfilter_can0[1].can_mask = CAN_SFF_MASK;
// 	rfilter_can0[2].can_id = 0x312;
//     rfilter_can0[2].can_mask = CAN_SFF_MASK;

//     setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_can0, sizeof(rfilter_can0));

//     return CC_OK;
// }

// 外部调用函数，获取车辆信息，包含车速与转向
// TODO: 后续可封装成为结构体，从车身CAN上获取更多车辆信息
STATE ROEWEControl::get_vehicle_info(float* car_speed, float* car_angle){
    *car_speed = car_speed_;
    *car_angle = car_angle_;
    return CC_OK;
}

// 外部调用函数，控制车辆函数，输入速度扭矩与角度扭矩，控制速度与转向
STATE ROEWEControl::send_vehicle_control_info(float& speed_torque, float& angle_torque){
    speed_torque_ = speed_torque;
    angle_torque_ = angle_torque;
    return CC_OK;
}

STATE ROEWEControl::enable_vehicle_control(bool enable_control){
    enable_control_ = enable_control;
    return CC_OK;
}

// 角度控制输出
STATE ROEWEControl::send_m_VMS2EPS(VCI_CAN_OBJ * frame, float& angle_torque, unsigned char enable_control){
    frame->ID = 0x310;
	frame->DataLen = 8;

	unsigned short SteeringTorque_Demand = (angle_torque - (-10)) / 0.01;

	VMS2EPS m_VMS2EPS;

	unsigned char EPS_remote_control = enable_control << 7;
	m_VMS2EPS.SteeringTorque_Demand[1] = SteeringTorque_Demand;
	m_VMS2EPS.SteeringTorque_Demand[0] = SteeringTorque_Demand >> 8;

	unsigned char tmp1 = 0x00;
	tmp1 = tmp1 + EPS_remote_control + m_VMS2EPS.SteeringTorque_Demand[0];
	frame->Data[0] = tmp1;
	frame->Data[1] = m_VMS2EPS.SteeringTorque_Demand[1];

    return CC_OK;
}

// 速度控制输出
STATE ROEWEControl::send_m_MAB2VCU(VCI_CAN_OBJ * frame, float& speed_torque, unsigned char enable_control){
	frame->ID = 0x313;
	frame->DataLen = 8;

	unsigned short MotorTorque_Req = (speed_torque - (-512)) / 0.5;

	MAB2VCU m_MAB2VCU;

	unsigned char Motor_remote_control = enable_control << 7;

	m_MAB2VCU.MotorTorque_Req[1] = MotorTorque_Req;
	m_MAB2VCU.MotorTorque_Req[0] = MotorTorque_Req >> 8;

	unsigned char tmp1 = 0x00;
	tmp1 = tmp1 + Motor_remote_control + m_MAB2VCU.MotorTorque_Req[0];

	frame->Data[0] = tmp1;
	frame->Data[1] = m_MAB2VCU.MotorTorque_Req[1];

    return CC_OK;
}

// 方向盘转向获取
STATE ROEWEControl::get_m_EPS_HSC_FrP01(VCI_CAN_OBJ *frame, float* car_angle){
    
    EPS_HSC_FrP01 m_EPS_HSC_FrP01;
	/*********SteeringAngleHSC1************/
	m_EPS_HSC_FrP01.SteeringAngleHSC1[0] = frame->Data[1];
	m_EPS_HSC_FrP01.SteeringAngleHSC1[1] = frame->Data[0];
	unsigned short tmp1 = m_EPS_HSC_FrP01.SteeringAngleHSC1[1];
	tmp1 = tmp1 << 8;
	unsigned short SteeringAngleHSC1_short = tmp1 + m_EPS_HSC_FrP01.SteeringAngleHSC1[0];

    // 需要输出的结果，方向盘转角
	float SteeringAngleHSC1 = SteeringAngleHSC1_short * 0.1 + (-3277);
    *car_angle = SteeringAngleHSC1;
	
	/*********SteeringVelocityHSC1************/
	m_EPS_HSC_FrP01.SteeringVelocityHSC1[0] = frame->Data[3];
	m_EPS_HSC_FrP01.SteeringVelocityHSC1[1] = frame->Data[2];
	unsigned short tmp2 = m_EPS_HSC_FrP01.SteeringVelocityHSC1[1];
	tmp2 = tmp2 << 8;
	unsigned short SteeringVelocityHSC1_short = tmp2 + m_EPS_HSC_FrP01.SteeringVelocityHSC1[0];
    
	short SteeringVelocityHSC1 = SteeringVelocityHSC1_short * 1 + (-32768);


	m_EPS_HSC_FrP01.UB_EPSStatus = frame->Data[4] >> 3;
	unsigned char UB_EPSStatus = m_EPS_HSC_FrP01.UB_EPSStatus;

	m_EPS_HSC_FrP01.EPSStatusHSC1 = frame->Data[4] >> 4;
	unsigned char EPSStatusHSC1 = m_EPS_HSC_FrP01.EPSStatusHSC1;

	m_EPS_HSC_FrP01.SteeringVelocityValidHSC1 = frame->Data[4] >> 6;
	unsigned char SteeringVelocityValidHSC1 = m_EPS_HSC_FrP01.SteeringVelocityValidHSC1;

	m_EPS_HSC_FrP01.SteeringAngleValidHSC1 = frame->Data[4] >> 7;
	unsigned char SteeringAngleValidHSC1 = m_EPS_HSC_FrP01.SteeringAngleValidHSC1;
	return 0;
}

// 车速获取
STATE ROEWEControl::get_m_VCU2MAB_2(VCI_CAN_OBJ * frame,  float* car_speed){
    VCU2MAB_2 m_VCU2MAB_2;

	m_VCU2MAB_2.Motor_control_mode = frame->Data[0] >> 7;
	unsigned char Motor_control_mode = m_VCU2MAB_2.Motor_control_mode;
	//INFO("Motor_control_mode:" << (int)Motor_control_mode);

	frame->Data[0] = frame->Data[0] & 0x07;
	m_VCU2MAB_2.MotorTorque[0] = frame->Data[1];
	m_VCU2MAB_2.MotorTorque[1] = frame->Data[0];
	unsigned short tmp1 = m_VCU2MAB_2.MotorTorque[1];
	tmp1 = tmp1 << 8;
	unsigned short MotorTorque_short = tmp1 + m_VCU2MAB_2.MotorTorque[0];
	float MotorTorque = MotorTorque_short * 0.5 + (-512);
	//INFO("MotorTorque:" << MotorTorque);

	m_VCU2MAB_2.MotorSpeed[0] = frame->Data[3];
	m_VCU2MAB_2.MotorSpeed[1] = frame->Data[2];
	unsigned short tmp2 = m_VCU2MAB_2.MotorSpeed[1];
	tmp2 = tmp2 << 8;
	unsigned short MotorSpeed_short = tmp2 + m_VCU2MAB_2.MotorSpeed[0];
	short MotorSpeed = MotorSpeed_short * 1 + (-32767);
	//INFO("MotorSpeed:" << (int)MotorSpeed);

	m_VCU2MAB_2.VehicleSpeedHSC1[0] = frame->Data[5];
	m_VCU2MAB_2.VehicleSpeedHSC1[1] = frame->Data[4];
	unsigned short tmp3 = m_VCU2MAB_2.VehicleSpeedHSC1[1];
	tmp3 = tmp3 << 8;
	unsigned short VehicleSpeedHSC1_short = tmp3 + m_VCU2MAB_2.VehicleSpeedHSC1[0];

	float VehicleSpeedHSC1 = VehicleSpeedHSC1_short * 0.01 + 0;
	if(MotorSpeed < 0){
            *car_speed = -VehicleSpeedHSC1;
	}
	else{
            *car_speed = VehicleSpeedHSC1;
	}

    return CC_OK;
}

STATE ROEWEControl::get_m_EPS2VMS(VCI_CAN_OBJ * frame, unsigned char* control_mode){
	EPS2VMS m_EPS2VMS;

	m_EPS2VMS.EPS_control_mode = frame->Data[0] >> 6;
	// EPS控制位获取
	*control_mode = m_EPS2VMS.EPS_control_mode;
	DEBUG("CANINFO: control_mode:" << int(*control_mode));

	m_EPS2VMS.SteeringTorqueValidHSC1 = frame->Data[0] >> 7;
	unsigned char SteeringTorqueValidHSC1 = m_EPS2VMS.SteeringTorqueValidHSC1;

	frame->Data[0] = frame->Data[0] & 0x07;
	m_EPS2VMS.SteeringTorqueHSC1[0] = frame->Data[1];
	m_EPS2VMS.SteeringTorqueHSC1[1] = frame->Data[0];
	unsigned short tmp1 = m_EPS2VMS.SteeringTorqueHSC1[1];
	tmp1 = tmp1 << 8;
	unsigned short SteeringTorqueHSC1_short = tmp1 + m_EPS2VMS.SteeringTorqueHSC1[0];
	float SteeringTorqueHSC1 = SteeringTorqueHSC1_short *0.01 + (-10);

	m_EPS2VMS.EPS_MotorCurrent[0] = frame->Data[3];
	m_EPS2VMS.EPS_MotorCurrent[1] = frame->Data[2];
	unsigned short tmp2 = m_EPS2VMS.EPS_MotorCurrent[1];
	tmp2 = tmp2 << 8;
	unsigned short EPS_MotorCurrent_short = tmp2 + m_EPS2VMS.EPS_MotorCurrent[0];
	float EPS_MotorCurrent = EPS_MotorCurrent_short * 0.1 + (-100);

	m_EPS2VMS.MorotTorqueBack[0] = frame->Data[5];
	m_EPS2VMS.MorotTorqueBack[1] = frame->Data[4];
	unsigned short tmp3 = m_EPS2VMS.MorotTorqueBack[1];
	tmp3 = tmp3 << 8;
	unsigned short MorotTorqueBack_short = tmp3 + m_EPS2VMS.MorotTorqueBack[0];
	float MorotTorqueBack = MorotTorqueBack_short * 0.01 + (-20);

	return CC_OK;
}
