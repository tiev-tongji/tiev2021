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
#include "nature.h"
using namespace std;

#define Debug 1

#define SHOW(x) cout << #x << " = " << x+0 << "  "

EHBControl::EHBControl(){
	sendCount = 0;
    openCAN = false;
}

EHBControl::~EHBControl(){

}

void EHBControl::init(){

    rcv_buff_size = 1000;
    rcv_wait_time = 100;
    openCAN = true;

	INFO("Start to subscribe EHB info");
    static std::thread get_info(&EHBControl::canInfoRead, this);

	INFO("Start to publish EHB info");
    static std::thread send_info(&EHBControl::canInfoSend, this);

	return;
}

// 读取EHB制动信息
int EHBControl::canInfoRead(){
	while(true){
		int nbytes;
		VCI_CAN_OBJ frame[rcv_buff_size];
		uint32_t cnt = VCI_Receive(can_dev.devType, can_dev.devIndex, 
                          can_dev.channelNum, frame, rcv_buff_size, rcv_wait_time);
		// INFO("cnt1 = "<<cnt);
		for (int i = 0; i < cnt; i++)
		{
			if (frame[i].ID == 0x304)
			{
				get_m_EHB_TX2(&frame[i]);
				return 0; // htf: why return ?
			}
		}
		usleep(50 * 1000);
	}
	return -1;
}

void EHBControl::canInfoSend(){
	while(1){
		usleep(10*1000);
		if(dcuMessage_.AimPressure == 0){
			continue;
		}
		VCI_CAN_OBJ frame[2];
		frame[0].ID = 0x303;
		frame[0].DataLen = 8;
		send_m_TX2_EHB(frame);
		// frame[1] = frame[0];
		// std::time_t time_now = TiEV::getTimeStamp();
		// INFO("send ehb: "<<(int)frame[0].Data[4]);
		// VCI_Transmit(can_dev.devType, can_dev.devIndex, can_dev.channelNum, frame, 1);
		INFO("vci return = "<<VCI_Transmit(can_dev.devType, can_dev.devIndex, can_dev.channelNum, frame, 1));
		// INFO("time = "<<TiEV::getTimeStamp() - time_now);
	}
	return;
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

void EHBControl::get_m_EHB_TX2(VCI_CAN_OBJ *frame){
	ehbMessage_.EHBStatus = frame->Data[0] & 0x01;
	ehbMessage_.ParkingBrakeRequest = frame->Data[0] & 0x02;
	ehbMessage_.ActualPressure = frame->Data[1];
	ehbMessage_.BrakePedalTravel = frame->Data[3];
	ehbMessage_.EHBFaultCode = frame->Data[4];
	ehbMessage_.AimPressureAnswered = frame->Data[5];
	if(Debug == 1){
		cout<<"ehb info:  ";
		SHOW(ehbMessage_.EHBStatus);
		SHOW(ehbMessage_.ParkingBrakeRequest);
		SHOW(ehbMessage_.ActualPressure);
		cout << endl;
	}
}


void EHBControl::send_m_TX2_EHB(VCI_CAN_OBJ *frame){
	sendCount ++;
	if(sendCount == 16){
		sendCount = 0;
	}
	for(int i=0; i<8;i++){
		frame->Data[i] = 0;
	}
        if(dcuMessage_.AimPressure == 0)
        {
                dcuMessage_.ParkingBrakeActive = 0;
        }
        else
        {
                dcuMessage_.ParkingBrakeActive = 2;
        }
	frame->Data[0] = dcuMessage_.BrakingMode & 0x0F;
	frame->Data[0] = frame->Data[0] << 4;
	frame->Data[0] = frame->Data[0] | (dcuMessage_.ParkingBrakeActive & 0x0F);

	int vehicleSlope = (float)(dcuMessage_.VehicleSlope/0.176);
	frame->Data[1] = vehicleSlope & 0xFF;

	int vehicleSpeed = (float)(dcuMessage_.VehicleSpeed/0.01562);
	frame->Data[2] = vehicleSpeed & 0xFF;
	frame->Data[3] = (vehicleSpeed >> 8) & 0xFF;

	if(dcuMessage_.AimPressure < 0){
		dcuMessage_.AimPressure = 0;
	}
	else if(dcuMessage_.AimPressure > 100){
		dcuMessage_.AimPressure = 100;
	}

	frame->Data[4] = dcuMessage_.AimPressure & 0xFF;

	int vehicleLongitudinalAcc = (float)((dcuMessage_.VehicleLongitudinalAcc + 40.95)/0.2);
	frame->Data[5] = vehicleLongitudinalAcc & 0xFF;

	frame->Data[6] = sendCount & 0x0F;
	frame->Data[6] = frame->Data[6] << 4;
	frame->Data[6] = frame->Data[6] | (vehicleLongitudinalAcc >> 8) & 0x0F;

	unsigned char SumCheck = 0;
	for(int i=0; i<8-1;i++){
		SumCheck += frame->Data[i];
	}
	frame->Data[7] = SumCheck & 0xFF;

}


