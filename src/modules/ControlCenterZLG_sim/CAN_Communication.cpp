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
 * @LastEditTime: 2019-10-25 22:28:12
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <lcm/lcm-cpp.hpp>
#include "CAN_Message_Definition.h"
#include "structCANINFO.hpp"
#include "structCANCONTROL.hpp"
#include "structREMOTECONTROL.hpp"
#include "structACC.hpp"
#include "ControlInstruct.h"
#include <pthread.h>
#include "EHBControl.hpp"
#include "RemoteControl.hpp"
#include "zlg_can.h"



void *ReadCANInfo(void* arg);
void *KeyboardControl(void* arg);
EHBMessage EHBmessage;
DCUMessage DCUmessage;
EHBControl EHB;

void *SendCAN0Data(void* arg);
void send_m_ABS2HSC(VCI_CAN_OBJ * frame);
void get_m_EPS_HSC_FrP01(VCI_CAN_OBJ *frame);
void get_m_EPS2VMS(VCI_CAN_OBJ * frame);
void get_m_VCU2MAB_2(VCI_CAN_OBJ * frame);
void get_m_VCU2MAB_3(VCI_CAN_OBJ * frame);
void get_m_VCU2MAB_4(VCI_CAN_OBJ * frame);
void get_m_VCU2MAB_5(VCI_CAN_OBJ * frame);
void get_m_BSC2MCU(VCI_CAN_OBJ * frame);
void send_m_VMS2EPS(VCI_CAN_OBJ * frame, unsigned char EPS_remote_control_p, double SteeringTorque_Demand_p);
void send_m_MAB2VCU(VCI_CAN_OBJ * frame, unsigned char Motor_remote_control_p, double MotorTorque_Req_p);
void send_m_MCU2BSC(VCI_CAN_OBJ * frame, unsigned char BRK_URG_EN, unsigned char BRK_EN, double BRK_OBJ_PRUR_p);

void *Send_CANSpeed(void *arg);
void *Manual_Control(void *arg);
void *Center_Control(void *arg);

// LCM struct init
lcm::LCM lcm_car("udpm://239.255.76.67:7667?ttl=1");
structCANINFO can_info;
lcm::LCM lcm_car_sub("udpm://239.255.76.67:7667?ttl=1");

// pthread define
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;

bool bQuit = false;

/* Send and receive work flag */
bool GetCAN0WorkFlag[2];
bool GetCAN1WorkFlag[2];
bool SendCAN0WorkFlag[2];
bool SendCAN1WorkFlag[2];

/* Send enable and params */
double motor_torque = 0; 			// from -510 t0 510, accuracy 0.01
int motor_torque_enable = 1; 		// motor enable which 1 means enable
double steering_torque = 0;			// from -10 t0 10, accuracy 0.01
int steering_torque_enable = 1; 	// steer enable which 1 means enable
double breaking_object = 0; 		// from 0 t0 6, accuracy 0.01
int breaking_object_enable = 0; 	// break enable which 1 means enable


/* global params */
double Car_Speed = 0;				// car speed now
double Car_Angle = 0;				// car steer now
double Angle_target = 0;			// target car angle
double Speed_target = 0;			// target car speed

/* PID params */
// acc
double acc_P = 0.8;
double acc_I = 0.0;
double acc_D = 0.0;					// no use
double acc_T = 0.1;					// T is for intergal
// steer
double turn_P = 0.006;    			// 0.0065 12.6 0.006 9.9
double turn_I = 0.03;				// 0.027 12.6 9.9
double turn_D = 0.0000000; 			// 0.0000005 9.9
// break
double object_P = 0.15;
double object_I = 0.00;
double object_D = 0.00;

/************************************ Driver control*****************************************/
double D_speed_sum = 0;
// 
double D_speed_last = 0;
// 
long long int count_v = 0;
double oldTarget = 0;
// Car is forward or backward
int forwardOrBack = 1;
// output data (speed angle) read from CAN
std::ofstream fout("data.txt");

/************************** Auto control or Manually control**************************/
bool remoteControl = 0; 			// 1 for auto, 0 for Manual
bool goOrStop = 1; 					// 1 for go, 0 for stop
int recvControlCount = 0; 			// Not receieve Control info from LCM
int recvACCCount = 0; 				// Not receieve ACC info from LCM
enum { MANUAL = 0, AUTO, START, STOP };
unsigned char EPS_control_mode;
ControlInstruct controlTarget;
int accSpeed = 0; //acc???
float accSpeedForward = 0; //acc???
float accSpeedBackward = 0; //acc???

int Test_flag = 0; 
structCANCONTROL can_control;
void *TEST(void *arg)
{
	while(1)
	{
		if(Test_flag == 1)
		{
			lcm_car.publish("GUAGUA", &can_control);
			Test_flag = 0;
			//std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;
		}	
	}
	return (void *)0;
}

/*???????????*/
double PI_Speed(double V_target)
{

	/******************  params set *********************/
	double F_t = 0;					// Real_time output Torque
	double D_speed = 0;				// Real_time Error speed

	
	if (oldTarget * V_target < 0 && Car_Speed > 0)
	{
		V_target = 0;
	}

	if (V_target < 0 && Car_Speed == 0)
	{
		forwardOrBack = -1;
	}
	if (V_target > 0 && Car_Speed == 0)
	{
		forwardOrBack = 1;
	}

	Car_Speed *= forwardOrBack;
	D_speed = V_target - Car_Speed;
	oldTarget = V_target;
	

	if (D_speed < -15)
	{
		D_speed = -15;
	}
	if (D_speed > 15)
	{
		D_speed = 15;
	}

	if (fabs(D_speed) > 5)
		D_speed_sum = D_speed_sum + D_speed;
	else{
		D_speed_sum = 0;
	}

	double acc = 0;

	if (D_speed * Car_Speed >= 0)
	{

		breaking_object_enable = 0;
		breaking_object = 0;
		acc = acc_P * D_speed + acc_I * D_speed_sum;
	}
	else
	{
		breaking_object_enable = 1;

		acc = acc_P * D_speed * 1.2;
		if (fabs(Car_Speed) < 5)
		{
			breaking_object = 5;
		}
		else if (fabs(Car_Speed) < 20)
		{
			breaking_object = 10;
		}
		else
		{
			breaking_object = 15;
		}
		//acc = 0;
		breaking_object = ceil(-acc);
		cout <<  "Stop Acc: "<< acc << endl;
		cout <<  "breaking_object: "<< breaking_object << endl;
		DCUmessage.AimPressure = breaking_object;
	}

	D_speed_last = D_speed;

	//printf("ACC is:%d\n", acc);
	usleep(10*1000);
	if(breaking_object_enable == 0){
		F_t = 15 * acc;
	}
	else{
		F_t = 0;
	}
	return F_t;
}

/*******************************??????***************************************/
double D_angle_sum = 0;
//???????????
double D_Angle_last = 0;

/*?????????*/
double PI_Steer(double Angle_target)
{

	//std::cout << "!!!!!!!!!!!!Angle_target: " << Angle_target << std::endl;
	if (controlTarget.speed >= 40)
	{
	if (Angle_target - Car_Angle> 20)
	{
		Angle_target = Car_Angle+ 20;
	}
	else if(Angle_target - Car_Angle< -20)
	{
		Angle_target = Car_Angle- 20;
	}

	//Limit torque when High Speed
		if (Angle_target > 40)
		{
			Angle_target = 40;
		}
		else if(Angle_target < -40)
		{
			Angle_target = -40;
		}
	}
	//std::cout << "-----------Now Angle_target: " << Angle_target << std::endl;
	/*****????????PID???��???*****/
	/*
	//1?????????????????????????��????��?????????????
	//???????????????????????��??????????????
	//?????????????????????
	//2??????????????????????????��?????????????????
	//??????��?????????????????????????????????
	//??��?????80%???????????????��??????????????
	//??????
	//3??????????????????????????????��???????????
	//???��????��????��??��????????????????????
	*/

	// ???????
	timeval start,end,time;
	double timeuse;
	gettimeofday(&time, NULL);
	/******************************************???????????*********************************/

	// ???????  10ms
	double T = 0.01;
	//??????????
	double torque = 0;

	/*********************************?????????***********************************/

	//??????
	double D_Angle = Angle_target - Car_Angle;
	//printf("\rangle: %f Angle_target: %f D_Angle: %f", Car_Angle, Angle_target, D_Angle);
	/* ��???(1??)??????*/
	/*if (fabs(D_Angle) < 3)
	{
	D_Angle = 0;
	}*/

	//TODO: the limit should be canceled
	//if (fabs(D_Angle) > 40)
	//{
	//	if (D_Angle < 0)
	//	{
	//		D_Angle = -40;
	//	}
	//	else
	//	{
	//		D_Angle = 40;
	//	}
	//}

	//?????????????
	static int count = 0;
	if (Car_Speed == 0 || remoteControl == MANUAL)
	{
		count++; //??????0?????
	}
	//?????????????0???????��?
	if (Car_Speed < 1 || remoteControl == AUTO)
	{
		if (count > 10)
		{
			torque = turn_P * D_Angle + (D_Angle - D_Angle_last) * turn_D / T;
		}
		else
		{
			D_angle_sum = D_angle_sum + D_Angle;
			torque = turn_P * D_Angle + T * turn_I  * D_angle_sum + (D_Angle - D_Angle_last) * turn_D / T;
		}
	}
	else
	{
		count = 0;
		D_angle_sum = D_angle_sum + D_Angle;

		//printf("Car_Angle = %f \n",Car_Angle);
		//double anglediff = turn_D * (D_Angle - D_Angle_last) / T;
		//printf("Angle_diff = %f \n", anglediff);
		//torque = V1_Kp * D_Angle + T * V1_Ti  * D_angle_sum;
		torque = turn_P * D_Angle + T * turn_I  * D_angle_sum + (D_Angle - D_Angle_last) * turn_D / T;

	}

//	printf("                      D_Angle = %f \n", D_Angle);
	//printf("                  D_angle_sum = %f  \n", D_angle_sum);
	//???????????????��??????
//	printf(" Control_torque = %f  \n", torque);
	D_Angle_last = D_Angle;

	//std::cout << "-----------" << torque << std::endl;
	
	//compare with torque and steer
	/*
	if (torque - Car_Angle> 20)
	{
		torque = Car_Angle+ 20;
	}
	else if(torque - Car_Angle< -20)
	{
		torque = Car_Angle- 20;
	}
	*/
	//Limit torque when High Speed
	usleep(10000);
/*	if (Car_Speed <= 8) torque +=-0.080;
	else if (Car_Speed <=15) torque +=-0.084;
	else if (Car_Speed <=25) torque +=-0.084;
	else
	{	
		std::ifstream ifs("bias.txt");
		float bias;
		ifs >> bias;
		torque += bias;
	}
*/	return torque - 0.08;
}


/*?????????*/
void *Center_Control(void *arg)
{
	timeval time;
	char filename[20];
	gettimeofday(&time, NULL);
	
		while (1){
			if (goOrStop == 1)
			{
				Angle_target = controlTarget.steer;
				
				//controlTarget.speed = 5;
				if( controlTarget.speed >= 0)
                                {
                                	Speed_target = controlTarget.speed < accSpeedForward ? controlTarget.speed : accSpeedForward;
                                }
                                else
                                {
                                	Speed_target = controlTarget.speed > accSpeedBackward ? controlTarget.speed : accSpeedBackward;
                                }
                                	//printf("Speed_target is %lf, accSpeedForward is %lf, accSpeedBackward is %lf\n", Speed_target, accSpeedForward,accSpeedBackward);
				//Speed_target = controlTarget.speed < accSpeed ? controlTarget.speed : accSpeed;
				//Speed_target = 5;
			}
			else
			{
				Angle_target = 0;
				Speed_target = 0;
			}
			//Speed_target = 10;
			//GetLocalTime(&sys_time);
			//fprintf(stderr, "%02d:%02d:%02d.%03d speed:%d steer:%d timestamp:%s\n", sys_time.wHour, sys_time.wMinute, sys_time.wSecond, sys_time.wMilliseconds,last.speed,last.steer,last.timestamp);
			if (EPS_control_mode == 0)
			{
				Speed_target = 0;
				std::cout << "Waiting for EPS control ..." << std::endl;
			}
			

			if(ehbLoss == 1 && Speed_target > 5){
				Speed_target = 5;
			}


			motor_torque = PI_Speed(Speed_target);
			steering_torque = PI_Steer(Angle_target);
			//printf("\rSPEED: %f  aimV: %f angle: %f aimR: %f", Car_Speed, Speed_target, Car_Angle, Angle_target);
			if (EPS_control_mode == 1)
			{
				printf("\rSPEED: %f  aimV: %f angle: %f aimR: %f\n", Car_Speed, Speed_target, Car_Angle, Angle_target);
				fout << Car_Speed << " " << Speed_target << " " << Car_Angle << " " << Angle_target << std::endl;
			}
			else
			{
				Speed_target = 0;
				std::cout << "Waiting for EPS control ..." << std::endl;
			}
			pthread_mutex_lock( &mutex1 );
			recvControlCount ++;
			pthread_mutex_unlock( &mutex1 );
			
			pthread_mutex_lock( &mutex2 );
			recvACCCount ++;
			pthread_mutex_unlock( &mutex2 );

			//cout << recvControlCount << endl;
			if (recvControlCount > 100 || recvACCCount > 100) //???8??????????????ACC???,?????;
			{
				goOrStop = 0;
			}
			else
			{
				goOrStop = 1;
			}
			//std::cout<<"go_Flag:  "<<goOrStop<<std::endl;
			usleep(10000);
	}
}


void *Send_CANSpeed(void *arg)
{
	while (1)
	{
		struct timeval tv;
    	gettimeofday(&tv,NULL);

		int steer = Car_Angle;
		int speed = (int)(Car_Speed * 100);
		if (forwardOrBack == -1 && speed >0){
			speed = -speed;
		}
		
		can_info.carspeed = speed;
		can_info.carsteer = steer;

		can_info.timestamp = tv.tv_sec*1000000 + tv.tv_usec;
		
		lcm_car.publish("CANINFO", &can_info);
		//std::cout << "Publish can info, speed is  " << speed << " steer is " << steer << std::endl;	

		usleep(10000);
	}
}

/* Set params manual */
void *Manual_Control(void *arg)
{
	
	while (1)
	{
		int c;
		std::cin >> c;
		//Angle_target = 90;
		switch (c)
		{
			case 97: turn_P += 0.001;   break;
			case 100: turn_P -= 0.001;  break;
			case 102: turn_I += 0.001;   break;
			case 103: turn_I -= 0.001;  break;
			//case 115: Speed_target = -10;  break;
			case 27: bQuit = true; std::cout << "quit" << std::endl; break;
			default:
				break;
		}
		/*cout << steer << " " << speed << endl;*/
		//Angle_target = steer;
		//cout << "speed_target"<<Speed_target << endl;
		usleep(10000);
	}
}

class Handler 
{
    public:
        ~Handler() {}

        void RecvCanControl(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const structCANCONTROL* msg)
        {
			pthread_mutex_lock( &mutex1 );
			recvControlCount = 0;
			pthread_mutex_unlock( &mutex1 );
			Test_flag = 1;
			
			can_control.timestamp = msg->timestamp;
			can_control.aimspeed = msg->aimspeed;
			can_control.aimsteer = msg->aimsteer;
			controlTarget.speed = msg->aimspeed;
			controlTarget.steer = msg->aimsteer;
			//std::cout << "!!!!!!SPEED is " << controlTarget.speed << std::endl;
			//std::cout << "!!!!!!STEER is " << controlTarget.steer << std::endl;
			if (controlTarget.speed> 60)
			controlTarget.speed = 60;
			/*if(controlTarget.speed > 5)
				controlTarget.speed= 5;*/
			//cout << controlTarget.speed << " " << controlTarget.steer << endl;
			return;
        }
		
		
		void RecvACC(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const structACC* msg)
        {
			pthread_mutex_lock( &mutex2 );
			recvACCCount = 0;
			pthread_mutex_unlock( &mutex2 );
			
			accSpeedForward = msg->ACCspeedforward;
			accSpeedBackward = msg->ACCspeedbackward;
			
			return;
        }
		
		
		void RecvRemoteControl(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const structREMOTECONTROL* msg)
        {
			uint8_t temp;
			temp = msg->enabled;
			if (temp == MANUAL)
			{
				remoteControl = 0;
			}
			if (temp == AUTO)
			{
				remoteControl = 1;
			}
			//std::cout << "REMOTECONTROL is " << remoteControl << std::endl;
			/*
			if (temp == STOP)
			{
				goOrStop = 0;
			}
			if (temp == START)
			{
				goOrStop = 1;
			}
			*/
			return;
        }
		
		
};

void *LCMThread(void *arg)
{
	if(!lcm_car.good()) return 0;
	if(!lcm_car_sub.good()) return 0;
	//Handler handlerRecvCanControl;
	//Handler handlerRecvACC;
	//Handler handlerRecvRemoteControl;
	Handler handlerObj;	

	lcm_car_sub.subscribe("CANCONTROL", &Handler::RecvCanControl, &handlerObj);
	lcm_car_sub.subscribe("ACC", &Handler::RecvACC, &handlerObj);
	lcm_car_sub.subscribe("REMOTECONTROL", &Handler::RecvRemoteControl, &handlerObj);
	while(0 == lcm_car_sub.handle());
}

int main()
{
	system("bash ../can_init.sh");

	remoteControl RemoteControl;
	int Status = RemoteControl.GPIO_Init();
	if(Status < 0){
		cout << "Init GPIO ERROR! EXIT" << endl;
		exit(1);
	}

	
	int C1,C2;
	struct sockaddr_can addr_can0;
	struct sockaddr_can addr_can1;
	struct ifreq ifr_can0;
	struct ifreq ifr_can1;
	struct can_filter rfilter_can0[6];
	struct can_filter rfilter_can1[1];	
	C1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	C2 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	
	strcpy(ifr_can0.ifr_name, "can0");
	strcpy(ifr_can1.ifr_name, "can1");
	ioctl(C1, SIOCGIFINDEX, &ifr_can0);
	ioctl(C2, SIOCGIFINDEX, &ifr_can1);
	addr_can0.can_family = AF_CAN;
	addr_can0.can_ifindex = ifr_can0.ifr_ifindex;
	addr_can1.can_family = AF_CAN;
	addr_can1.can_ifindex = ifr_can1.ifr_ifindex;
	
	bind(C1, (struct sockaddr *)&addr_can0, sizeof(addr_can0));
	bind(C2, (struct sockaddr *)&addr_can1, sizeof(addr_can1));
	
	rfilter_can0[0].can_id = 0x18B;
    rfilter_can0[0].can_mask = CAN_SFF_MASK;
	rfilter_can0[1].can_id = 0x312;
    rfilter_can0[1].can_mask = CAN_SFF_MASK;
	rfilter_can0[2].can_id = 0x302;
    rfilter_can0[2].can_mask = CAN_SFF_MASK;
	rfilter_can0[3].can_id = 0x303;
    rfilter_can0[3].can_mask = CAN_SFF_MASK;
	rfilter_can0[4].can_id = 0x304;
    rfilter_can0[4].can_mask = CAN_SFF_MASK;
	rfilter_can0[5].can_id = 0x305;
    rfilter_can0[5].can_mask = CAN_SFF_MASK;
	
	rfilter_can1[0].can_id = 0x101;
    rfilter_can1[0].can_mask = CAN_SFF_MASK;

	setsockopt(C1, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_can0, sizeof(rfilter_can0));
	setsockopt(C2, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_can1, sizeof(rfilter_can1));
	
	GetCAN0WorkFlag[0] = true;
	GetCAN1WorkFlag[0] = true;
	SendCAN0WorkFlag[0] = true;
	SendCAN1WorkFlag[0] = true;
	GetCAN0WorkFlag[1] = true;
	GetCAN1WorkFlag[1] = true;
	SendCAN0WorkFlag[1] = true;
	SendCAN1WorkFlag[1] = true;

	pthread_t id_LCMThread;
	int ret;
	ret=pthread_create(&id_LCMThread,NULL,LCMThread,NULL);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}
	
	pthread_t id_GetCAN0Data;
	pthread_t id_GetCAN1Data;
	pthread_t id_SendCAN0Data;
	pthread_t id_SendCAN1Data;

	pthread_t id_SendCarSpeed;
	pthread_t id_MaunalControl;
	pthread_t id_CenterControl;

	int *attr_G0=&C1;
	ret=pthread_create(&id_GetCAN0Data,NULL,GetCAN0Data,(void *)attr_G0);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}

	int *attr_G1=&C1;
	ret=pthread_create(&id_GetCAN1Data,NULL,GetCAN1Data,(void *)attr_G1);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}

	int *attr_S0=&C1;
	ret=pthread_create(&id_SendCAN0Data,NULL,SendCAN0Data,(void *)attr_S0);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}
	
	ret=pthread_create(&id_CenterControl,NULL,Center_Control,NULL);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}

	ret=pthread_create(&id_SendCarSpeed,NULL,Send_CANSpeed,NULL);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}
/*
	ret=pthread_create(&id_MaunalControl,NULL,Manual_Control,NULL);
	if(ret!=0)
	{
		std::cout << "Create pthread error!" << std::endl;
		return -1;
	}
*/

        pthread_t id_canRead;
        ret=pthread_create(&id_canRead,NULL,ReadCANInfo,(void *)NULL);
        pthread_t id_keyboard;
        ret=pthread_create(&id_keyboard,NULL,KeyboardControl,(void *)NULL);


	while (true)
	{
		int gpioValue = RemoteControl.GetGPIOValue();
		if(gpioValue < 0){
			cout << "GPIO Read ERROR!" << endl;
			exit(1);
		}

		if(gpioValue){
			DCUmessage.AimPressure = 30;
			EHB.canInfoSend();
			usleep(100*1000);
			cout << "Emergency Stop Car from remote control" << endl;
		}
		else if(remoteControl && breaking_object_enable){
		//if(1){
			EHB.sendDCUMessage(DCUmessage);
			EHB.canInfoSend();
			usleep(100*1000);
			cout << "---------DCUmessage: " << DCUmessage.AimPressure << endl;
		}

		if (bQuit)
		{
			int i = 1;
			break;
		}
		usleep(20*1000);
	}
	breaking_object_enable = 0;  // 
	usleep(100000);
	
	GetCAN0WorkFlag[0] = false;
	GetCAN1WorkFlag[0] = false;
	SendCAN0WorkFlag[0] = false;
	SendCAN1WorkFlag[1] = false;
	while (GetCAN0WorkFlag[1])
		usleep(1000);
	while (GetCAN1WorkFlag[1])
		usleep(1000);
	while (SendCAN0WorkFlag[1])
		usleep(1000);
	while (SendCAN1WorkFlag[1])
		usleep(1000);
	
	close(C1);
	close(C2);
	return 0;
	
}


void *ReadCANInfo(void* arg){
        while(1){
                //printf("Read CAN Info\n");
                EHB.canInfoRead();
        //      printf("Read CAN Info\n");
                usleep(20*1000);
        }
}

void *KeyboardControl(void* arg){
        while(0){
                EHB.keyboardControl();
                usleep(10*1000);
        }
}

bool ehbLoss = 0;

// Add ehb check function
/* receive CAN0 data pthead */
void *GetCAN1Data(void *arg)
{
	static ehbLossCount = 0;
	while(1){
		int res = EHB.canInfoRead();
		if(res != 0){
			ehbLossCount++;
		}
		else{
			ehbLossCount = 0;
		}

		if(ehbLossCount > 10*100){		// 10s
			//pthread_mutex_lock( &speedTargetSet );
			ehbLoss = 1;
			//speed_target = 5;
			//pthread_mutex_unlock( &speedTargetSet );
		}
		else{
			ehbLoss = 0;
		}
		usleep(10*1000);	// 100hz
	}
}
