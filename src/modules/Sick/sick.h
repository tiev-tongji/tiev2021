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
#include <arpa/inet.h>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <netdb.h>
#include <zcm/zcm-cpp.hpp>
#include "msg/include/structSICKMAP.hpp"
#include "msg/include/structNAVINFO.hpp"
#include "common/nature.h"

using namespace std;

namespace TiEV
{
	//zcm
	zcm::ZCM        myzcm{"ipc"};
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

	const int SICK_DA_LEN = 761;
	const int SICK_FILT = 20;
	const int SICK_FILT_SA = 40;
	const int SICK_FILT_SD = 500;
	const double SICK_TS_OBDIS = 0.200;
	const double SICK_PER_ANGLE = 0.5;
	const double SICK_ANGLE = 190;
	const double CABIRATE_SICK_ANGLE = 180;

	void SplitString(const string& s, vector<string>& v, const string& c)
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

	class Sick
	{
	private:
		int sock_fd;
		sockaddr_in server_addr;
		char buf[80000];
		int obds[SICK_DA_LEN];
	public:
		void init_client(char *ip, int port)
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
			char send_buf[] = "\2sEN LMDscandata 1\3";
			if (send(sock_fd, send_buf, strlen(send_buf), 0) != strlen(send_buf)) {
				perror("error in send");
				return;
			}
		}

		void start_recieve_client()
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
				pos_mutex.unlock();
				//
				DecodeSick();
			}
		}

		void DecodeSick()
		{
			//lcm
			mapData.resolution = TiEV::GRID_RESOLUTION;
			mapData.rows = TiEV::GRID_ROW;
			mapData.cols = TiEV::GRID_COL;
			mapData.center_col = TiEV::CAR_CEN_COL;
			mapData.center_row = TiEV::CAR_CEN_ROW;
			//
			string strSick = buf;
			// cout << "buf[0]" << buf << endl;
			auto loc = strSick.find("DIST");
			if (loc == string::npos)
			{
				cout << "no DIST in message" << endl;
				return;
			}
			memset(obds, 0, sizeof(int) * SICK_DA_LEN);
			strSick.erase(0, loc);
			vector<string> ar;
			SplitString(strSick, ar, " ");
			
			int i;
			int len = ar.size();
			for (i = SICK_FILT; i < SICK_DA_LEN - SICK_FILT&&i+6<len; i++)
			{
				//obds[i] = ToInt32(ar[i + 6]);
				// try{
				obds[i] = strtol(ar[i + 6].c_str(), NULL, 16);

					// throw 1;
				// }
				
				// catch(int a){
				// 	cout<<"split"<<endl;
				// 	obds[i] = 1;
				// }
				if (i < SICK_FILT_SA || i > SICK_DA_LEN - SICK_FILT_SA - 1 && obds[i] < SICK_FILT_SD)
					obds[i] = 0;
			}
			// cout<<"split"<<endl;
			//filter out wierd poind
			float Point_X[721] = { 0 };
			float Point_Y[721] = { 0 };
			double angle;
			for (int k = 0; k < 401; k++)
			{
				for (int j = 0; j < 149; j++)
				{
					mapData.cells[k][j] = 0;
				}
			}
			for (i = 40; i < 761; i++)
			{
				if (i == 760)
				{
					int j = 0;
					j++;
					j++;
					j++;
				}
				angle = (i * SICK_PER_ANGLE - (SICK_ANGLE - CABIRATE_SICK_ANGLE) / 2 + 90) / 180 * TiEV::TiEV_PI;
				// cout << angle;

				Point_X[i - 40] = (-(float)(obds[i] * cos(angle) / 1000)) * 2;
				Point_Y[i - 40] = ((float)(obds[i] * sin(angle) / 1000)) * 2;
				int CALIB_X = 289, CALIB_Y = mapData.cols / 2;
				int x = CALIB_X - (int)(Point_X[i - 40] * 5);
				int y = CALIB_Y + (int)(Point_Y[i - 40] * 5);
				
				if (x >= 0 && x < mapData.rows && y >= 0 && y < mapData.cols)
				{
					mapData.cells[x][y] = 1;
				}
			}
			//filter out wierd poind
			mapData.cells[289][75] = 0;
			mapData.cells[289][74] = 0;
			mapData.cells[288][73] = 0;
			myzcm.publish("SICKMAP", &mapData);
			memset(mapData.cells, 0, sizeof(mapData.cells));
			cout << "send Sick" << endl;
		}
	};
}
#endif /* sick_h */
