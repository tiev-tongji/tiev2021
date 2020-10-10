#include <zcm/zcm-cpp.hpp>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "structAIMPATH.hpp"
#include "structCANCONTROL.hpp"
#include "structCANINFO.hpp"
#include "structNAVINFO.hpp"
#include "visual.h"

using namespace std;

static structCANCONTROL can_control_msg;
static structAIMPATH path_msg;
static structCANINFO can_info_msg;
static structNAVINFO nav_info_msg;

static mutex control_mtx;
static mutex path_mtx;
static mutex can_mtx;
static mutex nav_mtx;

static Visualization visual;

class Handler{
	public:
		~Handler() {}

		void handleMessageFromCANCONTROL(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANCONTROL* msg)
		{
			control_mtx.lock();
			can_control_msg = *msg;
			control_mtx.unlock();
		}

		void handleMessageFromAIMPATH(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structAIMPATH* msg)
		{
			cout << "in this looooooooooooooooooooooooop" << endl;
			path_mtx.lock();
			path_msg = *msg;
			path_mtx.unlock();
		}

		void handleMessageFromCANINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structCANINFO* msg)
		{
			can_mtx.lock();
			can_info_msg = *msg;
			can_mtx.unlock();
		}

		void handleMessageFromNAVINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const structNAVINFO* msg)
		{
			nav_mtx.lock();
			nav_info_msg = *msg;
			nav_mtx.unlock();
		}
};

double curvature(const double a_x, const double a_y, const double b_x, const double b_y, const double c_x, const double c_y){
	double result;
	double dis1, dis2, dis3, dis;
	double cosA, sinA;

	dis1 = sqrt((a_x - b_x)*(a_x - b_x) + (a_y - b_y)*(a_y - b_y));
	dis2 = sqrt((a_x - c_x)*(a_x - c_x) + (a_y - c_y)*(a_y - c_y));
	dis3 = sqrt((c_x - b_x)*(c_x - b_x) + (c_y - b_y)*(c_y - b_y));
	dis = dis1*dis1 + dis3*dis3 - dis2*dis2;
	cosA = dis / (2*dis1*dis3);
	sinA = sqrt(1 - cosA*cosA);

	if(sinA == 0){
		result = 0;
	}
	else{
		double radius = dis2 / (2*sinA);
		result = 1 / radius;
	}
	return result;
}

void pub_msg(const int mode){
	zcm::ZCM zcm_pub{};
	if(!zcm_pub.good()){
		cout << "publish message zcm is not good" << endl;
		return ;
	}

	while(1)
	{
		structCANINFO can_info_test;
		structNAVINFO nav_info_test;
		structAIMPATH path_test;

		nav_info_test.mAngularRateZ = 0.2 + ((rand() % 100) - 50) / 1000.0;
		can_info_test.carsteer = can_control_msg.aimsteer;
		can_info_test.carspeed = can_control_msg.aimspeed;

//		srand(time(NULL));
		double velocity_desired = 5;
		TrajectoryPoint tmp[100];
		for(int i = 0; i < 100; i++){
			if(i == 0){
				tmp[i].x = 0.0;
				tmp[i].y = ((rand() % 50) - 25) / 1000.0;
				tmp[i].theta = 0.0;
				tmp[i].k = 0.0;
				tmp[i].v = 0.0;
				continue;
			}
			tmp[i].x = 0.1 * i;
			tmp[i].y = tmp[i-1].y + ((rand() % 50) - 25) / 1000.0;
			tmp[i].theta = atan((tmp[i].y - tmp[i-1].y) / (tmp[i].x - tmp[i-1].x)); 
//			tmp[i].v = velocity_desired + ((rand() % 500) - 250) / 1000.0;
			tmp[i].v = velocity_desired;
		}
		for(int i = 1; i < 99; i++){
			tmp[i].k = curvature(tmp[i-1].x, tmp[i-1].y, tmp[i].x, tmp[i].y, tmp[i+1].x, tmp[i+1].y); 
		}
		tmp[99].k = tmp[98].k;
		cout << tmp[99].x << "  " << tmp[99].y << "  " << tmp[99].theta * 180 / 3.1415926536 << "  " << tmp[99].k << "  " << tmp[99].v << endl;

		path_test.num_points = 100;
		for(int i = 0; i < path_test.num_points; i++){
			path_test.points.push_back(tmp[i]);
		}

		zcm_pub.publish("AIMPATH", &path_test);	
		zcm_pub.publish("CANINFO", &can_info_test);	
		zcm_pub.publish("NAVINFO", &nav_info_test);	

		usleep(20000);
	}
}

void sub_msg(const int mode){
	zcm::ZCM zcm_sub{};
	if(!zcm_sub.good()){
		cout << "subscribe message zcm is not good" << endl;
		return ;
	}
	
	Handler handlerObject;

	if(mode == 0){
		zcm_sub.subscribe("CANCONTROL", &Handler::handleMessageFromCANCONTROL, &handlerObject);

		while(0 == zcm_sub.handle());
	}
		
	if(mode == 1){
		zcm_sub.subscribe("AIMPATH", &Handler::handleMessageFromAIMPATH, &handlerObject);

		while(0 == zcm_sub.handle());
	}
}

void VisualThread()
{
	while(1){
		structAIMPATH path_test;
		TrajectoryPoint tmp[100];
		double velocity_desired = 5;

		for(int i = 0; i < 100; i++){
			if(i == 0){
				tmp[i].x = 0.0;
				tmp[i].y = ((rand() % 50) - 25) / 1000.0;
				tmp[i].theta = 0.0;
				tmp[i].k = 0.0;
				tmp[i].v = 0.0;
				continue;
			}
			tmp[i].x = 0.1 * i;
			tmp[i].y = tmp[i-1].y + ((rand() % 50) - 25) / 1000.0;
			tmp[i].theta = atan((tmp[i].y - tmp[i-1].y) / (tmp[i].x - tmp[i-1].x)); 
//			tmp[i].v = velocity_desired + ((rand() % 500) - 250) / 1000.0;
			tmp[i].v = velocity_desired;
		}
		for(int i = 1; i < 99; i++){
			tmp[i].k = curvature(tmp[i-1].x, tmp[i-1].y, tmp[i].x, tmp[i].y, tmp[i+1].x, tmp[i+1].y); 
		}
		tmp[99].k = tmp[98].k;

		path_test.num_points = 100;
		for(int i = 0; i < path_test.num_points; i++){
			path_test.points.push_back(tmp[i]);
		}

//		path_msg = path_test;

		cout << "zcm path size : " << path_msg.points.size() << endl;
		path_mtx.lock();
		visual.clearAimPath();
		visual.getAimPath(path_msg.points);
		path_mtx.unlock();

		usleep(20000);
	}
}

int main(int argc, char *argv[]){
	if(argc != 2){
		cout << "Usage: ./test_controller modenumber(0/1)" << endl;
		cout << "0: send virtual datas for test" << endl;
		cout << "1: show visualization and scopes" << endl;

		return 1;
	}

	int testmode = 0;

	if(!strcmp(argv[1], "0")){
		testmode = 0;

		thread task1(pub_msg, testmode);
		thread task2(sub_msg, testmode);
		task1.detach();
		task2.detach();

		while(1)
		{
			cout << "TimeStamp:" << can_control_msg.timestamp << " ----->"
				 << "  aimsteer = " << can_control_msg.aimsteer
				 << "  aimspeed = " << can_control_msg.aimspeed << endl;

			usleep(20000);
		}
	}

	if(!strcmp(argv[1], "1")){
		testmode = 1;

		thread task1(sub_msg, testmode);
		thread task2(VisualThread);
		task1.detach();
		task2.detach();

		cout << "test mode---------------------------->" << testmode << endl;
		visual.Show();
	}

	return 0;
}
