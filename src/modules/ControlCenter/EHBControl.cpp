/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: EHBControl.cpp
 * @Descripttion: EHB电子制动控制实现代码
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 18:48:52
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-11-09 21:32:05
 */

#include "EHBControl.hpp"
#include "ControlCenterCommon.h"
using namespace std;

#define _DEBUG 0

#define SHOW(x) cout << #x << " = " << x+0 << endl

EHBControl::EHBControl(){
	sendCount = 0;
    openCAN = false;
	struct sockaddr_can addr_can;
	struct ifreq ifr_can;
	CAN_PORT = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	strcpy(ifr_can.ifr_name, "can1");
	ioctl(CAN_PORT, SIOCGIFINDEX, &ifr_can);
	addr_can.can_family = AF_CAN;
	addr_can.can_ifindex = ifr_can.ifr_ifindex;

	bind(CAN_PORT, (struct sockaddr *)&addr_can, sizeof(addr_can));

	struct can_filter rfilter_can[2];
	//rfilter_can[0].can_id = 0x303;				//DCU
    //rfilter_can[0].can_mask = CAN_SFF_MASK;
    //rfilter_can[0].can_id = 0x304;				//EHB
    //rfilter_can[0].can_mask = CAN_SFF_MASK;
    setsockopt(CAN_PORT, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_can, sizeof(rfilter_can));

    openCAN = true;
}

EHBControl::~EHBControl(){

}

void EHBControl::init(){
	INFO("Start to subscribe EHB info");
    static std::thread get_info(&EHBControl::canInfoRead, this);

	INFO("Start to publish EHB info");
    static std::thread send_info(&EHBControl::canInfoSend, this);

	return;
}

// 读取EHB制动信息
int EHBControl::canInfoRead(){
	while(1){
		int nbytes;
		struct can_frame frame[1];
		//int rec = 50;
		nbytes = read(CAN_PORT, &frame, sizeof(frame));
		if ((frame)->can_id == 0x304)
		{
			get_m_EHB_TX2(frame);
			return 0;
		}
		usleep(50*1000);
	}
	return -1;
}

void EHBControl::canInfoSend(){
	while(1){
		usleep(50*1000);
		if(dcuMessage_.AimPressure == 0){
			continue;
		}
		can_frame frame[1];
		frame[0].can_id = 0x303;
		frame[0].can_dlc = 8;
		int nbytes;
		send_m_TX2_EHB(frame);
		nbytes = write(CAN_PORT, &frame[0], sizeof(frame[0]));
		if(nbytes != sizeof(frame))
		{
			std::cout << "CAN Send ERROR!" << std::endl;
		}
		
	}
}

void EHBControl::keyboardControl(){
	int c = getch();
	switch(c){
		case 'q':
			dcuMessage_.BrakingMode = 1;
			break;
		case 'a':
			dcuMessage_.BrakingMode = 2;
			break;
		case 'w':
			dcuMessage_.ParkingBrakeActive = 1;
			break;
		case 's':
			dcuMessage_.ParkingBrakeActive = 2;
			break;
		case 'e':
			dcuMessage_.AimPressure ++;
			break;
		case 'd':
			dcuMessage_.AimPressure --;
			break;
		default:
			break;
	}
	SHOW(dcuMessage_.BrakingMode);
	SHOW(dcuMessage_.ParkingBrakeActive);
	SHOW(dcuMessage_.AimPressure);
}

int EHBControl::getch(void){
	struct termios oldattr, newattr;  
	int ch;
	tcgetattr( STDIN_FILENO, &oldattr );  
	newattr = oldattr;  
	newattr.c_lflag &= ~( ICANON | ECHO );  
	tcsetattr( STDIN_FILENO, TCSANOW, &newattr );  
	ch = getchar();  
	tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );  
	return ch; 
}

EHBMessage EHBControl::getEHBMessage(){
	return ehbMessage_;
}

void EHBControl::sendDCUMessage(DCUMessage msg){
	dcuMessage_ = msg;
}

void EHBControl::get_m_EHB_TX2(can_frame *frame){
	ehbMessage_.EHBStatus = frame->data[0] & 0x01;
	ehbMessage_.ParkingBrakeRequest = frame->data[0] & 0x02;
	ehbMessage_.ActualPressure = frame->data[1];
	ehbMessage_.BrakePedalTravel = frame->data[3];
	ehbMessage_.EHBFaultCode = frame->data[4];
	ehbMessage_.AimPressureAnswered = frame->data[5];
	if(_DEBUG == 1){
		SHOW(ehbMessage_.EHBStatus);
		SHOW(ehbMessage_.ParkingBrakeRequest);
		SHOW(ehbMessage_.ActualPressure);
		cout << "~~~~~~~~~~~~~~~~~~~" << endl;
	}
}


void EHBControl::send_m_TX2_EHB(can_frame *frame){
	sendCount ++;
	if(sendCount == 16){
		sendCount = 0;
	}
	for(int i=0; i<8;i++){
		frame->data[i] = 0;
	}
	frame->data[0] = dcuMessage_.BrakingMode & 0x0F;
	frame->data[0] = frame->data[0] << 4;
	frame->data[0] = frame->data[0] | (dcuMessage_.ParkingBrakeActive & 0x0F);

	int vehicleSlope = (float)(dcuMessage_.VehicleSlope/0.176);
	frame->data[1] = vehicleSlope & 0xFF;

	int vehicleSpeed = (float)(dcuMessage_.VehicleSpeed/0.01562);
	frame->data[2] = vehicleSpeed & 0xFF;
	frame->data[3] = (vehicleSpeed >> 8) & 0xFF;

	if(dcuMessage_.AimPressure < 0){
		dcuMessage_.AimPressure = 0;
	}
	else if(dcuMessage_.AimPressure > 100){
		dcuMessage_.AimPressure = 100;
	}

	frame->data[4] = dcuMessage_.AimPressure & 0xFF;

	int vehicleLongitudinalAcc = (float)((dcuMessage_.VehicleLongitudinalAcc + 40.95)/0.2);
	frame->data[5] = vehicleLongitudinalAcc & 0xFF;

	frame->data[6] = sendCount & 0x0F;
	frame->data[6] = frame->data[6] << 4;
	frame->data[6] = frame->data[6] | (vehicleLongitudinalAcc >> 8) & 0x0F;

	unsigned char SumCheck = 0;
	for(int i=0; i<8-1;i++){
		SumCheck += frame->data[i];
	}
	frame->data[7] = SumCheck & 0xFF;

}


