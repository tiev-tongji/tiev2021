/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: ROEWECANDefine.h
 * @Descripttion: 荣威E50 部分CAN协议结构体
 * @Author: Ding Yongchao
 * @version: V1.0
 * @Date: 2019-10-25 18:48:52
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-25 19:15:00
 */


// ROEWE E50 CAN Message Definition
// Message采用大端，即高位数据在低地址

/******** EPS_V0_3_DIAS Start ********/
//EPS_HSC_FrP01
struct EPS_HSC_FrP01
{
	unsigned char SteeringAngleHSC1[2];
	unsigned char SteeringVelocityHSC1[2];
	unsigned char SteeringAngleValidHSC1:1;
	unsigned char SteeringVelocityValidHSC1:1;
	unsigned char EPSStatusHSC1:2;
	unsigned char UB_EPSStatus:1;
};//m_EPS_HSC_FrP01


//EPS2VMS
struct EPS2VMS
{
	unsigned char SteeringTorqueValidHSC1:1;
	unsigned char EPS_control_mode:1;
	unsigned char SteeringTorqueHSC1[2];
	unsigned char EPS_MotorCurrent[2];
	unsigned char MorotTorqueBack[2];
};// m_EPS2VMS;

struct VMS2EPS
{
	unsigned char EPS_remote_control:1;
	unsigned char SteeringTorque_Demand[2];
};//m_VMS2EPS;
//// EPS_V0_3_DIAS Finish



/******** MAB_VMS_V0_5 Start ********/
//MAV2VCU
struct MAB2VCU
{
	unsigned char Motor_remote_control:1;
	unsigned char MotorTorque_Req[2];
};// m_MAB2VCU;

//VCU2MAB_2
struct VCU2MAB_2
{
	unsigned char Motor_control_mode:1;
	unsigned char MotorTorque[2];
	unsigned char MotorSpeed[2];
	unsigned char VehicleSpeedHSC1[2];
};// m_VCU2MAB_2;

//VCU2MAB_3
struct VCU2MAB_3
{
	unsigned char PedalPosnAccel;
	unsigned char PedalPosnBrake;
	unsigned char shifterPosn:4;
	unsigned char EPBStatus:2;
	unsigned char BMSPackSoCHSC1;
	unsigned char BMSPackVoltHSC1[2];
	unsigned char BMSStatusHSC1:4;
};// m_VCU2MAB_3;

//VCU2MAB_4
struct VCU2MAB_4
{
	unsigned char wheelSpeedFLHSC1[2];
	unsigned char wheelSpeedFRHSC1[2];
	unsigned char wheelSpeedRLHSC1[2];
	unsigned char wheelSpeedRRHSC1[2];
};// m_VCU2MAB_4;

//VCU2MAB_5
struct VCU2MAB_5
{
	unsigned char WheelRollingCountFLHSC1[2];
	unsigned char WheelRollingCountFRHSC1[2];
	unsigned char WheelRollingCountRLHSC1[2];
	unsigned char WheelRollingCountRRHSC1[2];
};// m_VCU2MAB_5;


// TODO: 原武大协议，目前暂时已经废弃，不再使用
//线控制动协议
//BSC2MCU
struct BSC2MCU   //主控制器到制动系统控制器
{
	unsigned char LS_PRUR[2];   //左路压力
	unsigned char RS_PRUR[2];   //右路压力
	unsigned char BRK_URG:4;    //紧急制动
	unsigned char BRK_STS:4;    //制动状态
};

//MCU2BSC
struct MCU2BSC    //制动系统控制器到主控制器
{
	unsigned char BRK_OBJ_PRUR[2];  //目标制动压力
	unsigned char BRK_URG_EN:1;     //紧急制动
	unsigned char BRK_EN:1;         //制动使能
};
