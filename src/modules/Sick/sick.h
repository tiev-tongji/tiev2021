/*
 * Copyright (c) 2018 TiEV (Tongji Intelligent Electric Vehicle).
 * 
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 * 
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
//
//  sick.h
//  sick
//
//  Created by 高昕宇 on 2017/9/28. modified by 吴岩 赵君峤 on 2017/10/20.
//  Copyright © 2017年 Rizia. All rights reserved.
//

#ifndef sick_h
#define sick_h
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <arpa/inet.h>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <netdb.h>
#include <zcm/zcm-cpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "msg/include/structSICKMAP.hpp"
#include "msg/include/structNAVINFO.hpp"
#include "common/nature.h"

using namespace std;

#define COE_POS 1		  //系数位置
#define ANGLE_START_POS 3 //起始角度位置
#define ANGLE_STEP_POS 4  //角分辨率位置
#define DATA_NUM_POS 5	  //数据总数位置
#define DATA_START_POS 6  //数据起始位置

#define Y_SHIFT_FRONT -1 //前sick雷达Y轴偏移量
#define Y_SHIFT_BACK 3.2 //后sick雷达Y轴偏移量

namespace TiEV
{
	//zcm
	zcm::ZCM myzcm{"ipc"};
	// lcm::LCM mylcm("udpm://239.255.76.67:7667?ttl=1");
	structSICKMAP mapData;
	structNAVINFO currentPose;
	std::mutex pos_mutex;

	class Handler
	{
	public:
		~Handler() {}

		void handleNAVINFOMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structNAVINFO *msg){};
	};

	void SplitString(const string &s, vector<string> &v, const string &c)
	{
		string::size_type pos1, pos2;
		pos2 = s.find(c);
		pos1 = 0;
		while (string::npos != pos2)
		{
			v.push_back(s.substr(pos1, pos2 - pos1));

			pos1 = pos2 + c.size();
			pos2 = s.find(c, pos1);
		}
		if (pos1 != s.length())
			v.push_back(s.substr(pos1));
	}
	void sickmap_init()
	{
		//lcm
		mapData.resolution = TiEV::GRID_RESOLUTION;
		mapData.rows = TiEV::GRID_ROW;
		mapData.cols = TiEV::GRID_COL;
		mapData.center_col = TiEV::CAR_CEN_COL;
		mapData.center_row = TiEV::CAR_CEN_ROW;

		for (int k = 0; k < TiEV::GRID_ROW; k++)
		{
			for (int j = 0; j < TiEV::GRID_COL; j++)
			{
				mapData.cells[k][j] = 0;
			}
		}
	}
	class Sick
	{
	private:
		int sock_fd;
		sockaddr_in server_addr;
		char buf[80000];
		int obds[1000];

	public:
		void init_client(char *ip, int port) //将sick雷达链接初始化
		{
			if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
			{
				perror("ERROR IN SOCKET()");
				exit(1);
			}
			int yes;
			if ((setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1))
			{
				perror("ERROR IN SETSOCKOPT()");
				exit(2);
			}
			server_addr.sin_family = AF_INET;
			server_addr.sin_port = htons(port);
			server_addr.sin_addr.s_addr = inet_addr(ip);
			memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));
			// cout << "before connect" << endl;
			if (connect(sock_fd, (sockaddr *)&server_addr, sizeof(server_addr)) < 0)
			{
				perror("connect");
				exit(3);
			}
			// cout << "after connect" << endl;
			char send_buf[] = "\2sEN LMDscandata 1\3"; //连续测量：发送指令后，LMS 会实时返回其测量数据
													   //指令 HEX: 02 73 45 4E 20 4C 4D 44 73 63 61 6E 64 61 74 61 20 31 03
			if (send(sock_fd, send_buf, strlen(send_buf), 0) != strlen(send_buf))
			{
				perror("error in send");
				return;
			}
		}

		void start_recieve_client(bool kind) //kind = 0:front雷达代码解析，king=1:back雷达代码解析
		{
			int data_len = 0;
			memset(buf, 0, sizeof(buf));
			//TODO verify lens
			data_len = (int)recv(sock_fd, buf, 80000, 0);

			if (data_len <= 0)
			{
				perror("recv");
				return;
			}
			else
			{
				//zcm
				mapData.timestamp = TiEV::getTimeStamp();
				pos_mutex.lock();
				mapData.utmX = currentPose.utmX;
				mapData.utmY = currentPose.utmY;
				mapData.mHeading = currentPose.mHeading;
// 				mapData.utmX = 0;
// 				mapData.utmY = 0;
// 				mapData.mHeading = 0;
				pos_mutex.unlock();
				//
				DecodeSick(kind);
			}
		}

		void DecodeSick(bool kind) //kind = 0:front雷达代码解析，kind=1:back雷达代码解析
		{

			//
			string strSick = buf;
			auto loc = strSick.find("DIST");
			if (loc == string::npos)
			{
				cout << "no DIST in message" << endl;
				return;
			}
			memset(obds, 0, sizeof(int) * 761);
			strSick.erase(0, loc); //除掉字符串DIST前面的无用信息
			vector<string> ar;
			SplitString(strSick, ar, " "); //将数据进行处理，存入ar中

			int len = ar.size();
			cout << len << endl;

			for (int i = 0; i < len; i++)
			{
				obds[i] = strtoul(ar[i].c_str(), NULL, 16);
			}

			int coe = 1, data_num, start_angle; //距离倍数，数据总量,起始角度

			double angle, angle_step;

			if (ar[COE_POS] == "40000000")
				coe = 2;

			data_num = obds[DATA_NUM_POS];
			start_angle = obds[ANGLE_START_POS] / 10000;
			angle_step = double(obds[ANGLE_STEP_POS]) / 10000;
			if (kind)
				cout << "FRONT:";
			else
				cout << "BACK:";

			cout << "起始角度：" << start_angle << ' ' << "角分辨率：" << angle_step << " "
				 << "数据总量" << data_num << "距离系数" << coe << endl;
			vector<double> point_x;
			vector<double> point_y;

			for (int i = 0; i < data_num; i++)
			{
				angle = (i * angle_step + start_angle + kind * 180) / 180 * TiEV_PI;
				double x = double(obds[i + DATA_START_POS]) * cos(angle) * coe / 1000;
				double y = double(obds[i + DATA_START_POS]) * sin(angle) * coe / 1000;
				point_x.push_back(x);
				point_y.push_back(y);
			}
			if (!kind) //front雷达进行坐标变换
			{

				// 文件输出坐标
				/* ofstream fp_x, fp_y;
				fp_x.open("x_front.txt");

				for (int i = 0; i < point_x.size(); i++)
				{
					fp_x << point_x[i] << endl;
				}
				fp_y.open("y_front.txt");

				for (int i = 0; i < point_y.size(); i++)
				{
					fp_y << point_y[i] << endl;
				}*/
				for (int i = 0; i < data_num; i++)
				{
					int x = (int)(point_x[i] / TiEV::GRID_RESOLUTION) + TiEV::CAR_CEN_COL;
					int y = -(int)(point_y[i] / TiEV::GRID_RESOLUTION) + TiEV::CAR_CEN_ROW + Y_SHIFT_FRONT / TiEV::GRID_RESOLUTION;

					if (x >= 0 && x < mapData.cols && y >= 0 && y < mapData.rows)
					{
						mapData.cells[y][x] = 1;
					}
				}
			}
			else //back雷达进行坐标变换
			{

				// ofstream fp_x, fp_y;
				// fp_x.open("x_back.txt");

				// for (int i = 0; i < point_x.size(); i++)
				// {
				// 	fp_x << point_x[i] << endl;
				// }
				// fp_y.open("y_back.txt");

				// for (int i = 0; i < point_y.size(); i++)
				// {
				// 	fp_y << point_y[i] << endl;
				// }
				for (int i = 0; i < data_num; i++)
				{
					int x = (int)(point_x[i] / TiEV::GRID_RESOLUTION) + TiEV::CAR_CEN_COL;
					int y = -(int)(point_y[i] / TiEV::GRID_RESOLUTION) + TiEV::CAR_CEN_ROW + Y_SHIFT_BACK / TiEV::GRID_RESOLUTION;

					if (x >= 0 && x < mapData.cols && y >= 0 && y < mapData.rows)
					{
						mapData.cells[y][x] = 1;
					}
				}
			}
		}
	};
} // namespace TiEV
#endif /* sick_h */
