/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 18:48:52
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-11-09 21:35:37
 */

#ifndef EHBCONTROL_HPP
#define EHBCONTROL_HPP

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <termios.h> 
#include <atomic>
#include <thread>
#include "zlg_can.h"
#include "ControlCenterCommon.h"

using namespace std;

struct EHBMessage{
	unsigned char EHBStatus;
	unsigned char ParkingBrakeRequest;
	unsigned char ActualPressure;
	unsigned char BrakePedalTravel;
	unsigned char EHBFaultCode;
	unsigned char AimPressureAnswered;
};

struct DCUMessage{
	unsigned char BrakingMode = 0;
	unsigned char ParkingBrakeActive = 0;
	float VehicleSlope = 0.0;
	float VehicleSpeed = 0.0;
	unsigned char AimPressure = 0;
	float VehicleLongitudinalAcc = 0;
};

class EHBControl{
public:
	EHBControl();
	~EHBControl();

	void init();
	EHBMessage getEHBMessage();
	void sendDCUMessage(DCUMessage msg);
        int getCANPort(){//for ESR
		if(openCAN) return CAN_PORT;
		else return -1;
	}
private:
	int canInfoRead();
	void canInfoSend();
	void get_m_EHB_TX2(VCI_CAN_OBJ *frame);
	void send_m_TX2_EHB(VCI_CAN_OBJ *frame);
private:
	void keyboardControl();
	int getch();
	int CAN_PORT;
	bool openCAN;
	int sendCount;
	EHBMessage ehbMessage_;
	DCUMessage dcuMessage_;
public:
 	unsigned int rcv_wait_time;
    int rcv_buff_size;
    CAN_DEV_INFO can_dev;
    VCI_INIT_CONFIG config;
};

#endif
