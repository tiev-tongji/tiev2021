/*
 * Copyright (c) 2018 TiEV (Tongji Intelligent Electric Vehicle).
 * 
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 * 
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
//
//  Header.h
//  lux
//
//  Created by 高昕宇 on 2017/3/26, modified by 吴岩 管林挺 赵君峤 on 2020/10/20.
//  Copyright © 2020年 Rizia. All rights reserved.
//

#ifndef Header_h
#define Header_h
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include <vector>
#include <netdb.h>
#include <zcm/zcm-cpp.hpp>
#include "msg/include/structLUXMAP.hpp"
#include "msg/include/structNAVINFO.hpp"
#include "msg/include/MsgRainDetectionSignal.hpp"
#include "common/nature.h"

namespace TiEV
{
#define MY_PORT	7999
#define MY_IP	INADDR_ANY
#define LISTEN_NUM	1000
#define BUF_SIZE	80000
#define HEAD_LEN	24
//point type
#define FLAG_TRANSPARENT 0x01
#define FLAG_CLUTTER	0x02
#define FLAG_GROUD      0x04
#define FLAG_DIRT	0x08
#define TOL_RAIN_RATIO  0.17

	using namespace std;

	typedef unsigned char byte;

	ofstream ofs("/home/autolab/Desktop/TiEV/modules/Lux/angle.txt");///home/autolab/Desktop/TiEV/modules/Lux/tags
	//lcm
	zcm::ZCM myzcm_ipc{"ipc"};
	zcm::ZCM myzcm_udpm{"udpm://239.255.76.67:7667?ttl=1"};
	structLUXMAP mapData;
	structNAVINFO currentPose;
	std::mutex pos_mutex;

	class Handler
	{
		public:
			~Handler() {}
			void handleNAVINFOMessage(const zcm::ReceiveBuffer* rbuf,
					const std::string& chan,
					const structNAVINFO* msg);
	};

	class LPoint
	{
		public:
			byte ly;
			byte echo;
			byte flag;
			int16_t ha;
			uint16_t rd;
			uint16_t epw;
			uint16_t rev;
			double x;
			double y;
			double a;
			double d;
	};

	class LRRaw
	{
		public:
			uint64_t tm;
			uint16_t atpr;
			int16_t sa;
			int16_t ea;
			uint16_t npt;
			int16_t yaw;
			int16_t pitch;
			int16_t roll;
			int16_t mpx;
			int16_t mpy;
			int16_t mpz;
			uint16_t flag;
			vector<LPoint> lpts;
	};

	//
	class Laser_8
	{
		private:
			int sock_fd, Laser_tcp;
			sockaddr_in server_addr;
			sockaddr_in client_addr;
			fd_set fdsr;
			int conn_amount;
			int maxsock;
			int fd_a[LISTEN_NUM];
			int data_len;
			byte buf[BUF_SIZE];
			FILE *filestream;
			int num_obstacle_pts;
			int num_clutter_pts;//rain
			int num_dirt_pts;//dirts
		public:
			LRRaw lrRaw;
			double objmid = 1000;
			void init_client(char *ip, int port)
			{
				time_t cur_time;
				time(&cur_time);
				struct tm *timeinfo = localtime(&cur_time);
				char filename[20] = { 0 };
				//sprintf(filename, "LuxRaw%02d%02d%02d%02d.dat", timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
				//filestream = fopen(filename, "wb+");
				//if (filestream == NULL)
				//{
				//	perror("fopen");
				//	exit(6);
				//}
				conn_amount = 0;
				memset(fd_a, 0, LISTEN_NUM * sizeof(int));
				if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
				{
					perror("socket");
					exit(1);
				}
				int yes;
				if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
				{
					perror("setsockopt");
					exit(2);
				}
				server_addr.sin_family = AF_INET;
				server_addr.sin_port = htons(port);
				server_addr.sin_addr.s_addr = inet_addr(ip);
				memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

				if (connect(sock_fd, (sockaddr *)&server_addr, sizeof(server_addr)) < 0)
				{
					perror("connect");
					exit(3);
				}

			}

			void start_recieve_client()
			{
				data_len = 0;
				int MessageSize = 0;
				data_len = (int)recv(sock_fd, buf, 12, 0);
				if (data_len <= 0) {
					perror("recv");
					return;
				}
				else {
					while (1) {
						if (data_len < 12)
							data_len += recv(sock_fd, buf + data_len, 12 - data_len, 0);
						//fwrite(buf,sizeof(char),data_len, filestream);
						else if ((buf[0] == 0xAF && buf[1] == 0xFE && buf[2] == 0xC0 && buf[3] == 0xC2) && data_len < MessageSize + HEAD_LEN)
						{
							MessageSize = buf[8] << 24 | buf[9] << 16 | buf[10] << 8 | buf[11];
							if (data_len < MessageSize + HEAD_LEN)
								data_len += recv(sock_fd, buf + data_len, MessageSize + HEAD_LEN - data_len, 0);
						}
						else
							data_len = (int)recv(sock_fd, buf, 12, 0);
						if (data_len == MessageSize + HEAD_LEN) {
							//lcm
							mapData.timestamp = TiEV::getTimeStamp();
							pos_mutex.lock();
							mapData.utmX = currentPose.utmX;
							mapData.utmY = currentPose.utmY;
							mapData.mHeading = currentPose.mHeading;
							pos_mutex.unlock();
							//
							Decompose(MessageSize);
							data_len = 0;
						}
					}
				}
			}
			//TODO comment
			void init_server()
			{
				time_t cur_time;
				time(&cur_time);
				tm *timeinfo = localtime(&cur_time);
				char filename[20] = { 0 };
				//sprintf(filename, "data_%d-%d_%d:%d", timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
				//filestream = fopen(filename, "wb+");
				//if (filestream == NULL)
				//{
				//	perror("fopen");
				//	exit(6);
				//}
				conn_amount = 0;
				memset(fd_a, 0, LISTEN_NUM * sizeof(int));
				if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
				{
					perror("socket");
					exit(1);
				}
				int yes;
				if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
				{
					perror("setsockopt");
					exit(2);
				}
				server_addr.sin_family = AF_INET;
				server_addr.sin_port = htons(MY_PORT);
				server_addr.sin_addr.s_addr = MY_IP;
				memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));
				if (::bind(sock_fd, (sockaddr *)&server_addr, sizeof(server_addr)) == -1)
				{
					perror("bind");
					exit(3);
				}
				if (listen(sock_fd, LISTEN_NUM) == -1)
				{
					perror("listen");
					exit(4);
				}
			}
			//TODO comment
			void start_recieve_server()
			{
				timeval tv;
				unsigned int sin_size = sizeof(client_addr);
				int maxsock = sock_fd;
				int ret = 0;
				while (1)
				{
					FD_ZERO(&fdsr);
					FD_SET(sock_fd, &fdsr);
					tv.tv_sec = 30;
					tv.tv_usec = 0;
					for (int i = 0; i < LISTEN_NUM; i++)
						if (fd_a[i])
							FD_SET(sock_fd, &fdsr);
					ret = select(maxsock + 1, &fdsr, NULL, NULL, &tv);
					if (ret < 0)
					{
						perror("select");
						break;
					}
					else if (ret == 0)
					{
						continue;
					}
					else
					{
						for (int i = 0; i < conn_amount; i++)
						{
							if (FD_ISSET(fd_a[i], &fdsr))
							{
								ret = (int)recv(fd_a[i], buf, sizeof(buf), 0);
								if (ret <= 0)
								{
									close(fd_a[i]);
									FD_CLR(fd_a[i], &fdsr);
									fd_a[i] = 0;
								}
								else
								{
									//fwrite(buf, sizeof(char), ret, filestream);
								}
							}
						}
						int new_fd = 0;
						if (FD_ISSET(sock_fd, &fdsr))
						{
							new_fd = accept(sock_fd, (sockaddr *)(&client_addr), (unsigned int *)&sin_size);
							if (new_fd <= 0)
							{
								perror("accept");
								continue;
							}
						}
						if (conn_amount < LISTEN_NUM)
						{
							fd_a[conn_amount++] = new_fd;
							if (new_fd > maxsock)
								maxsock = new_fd;
						}
						else
							close(new_fd);
					}

				}
			}

			void Decompose(int MessageSize)
			{
				int i;
				for (i = 0; i < data_len - 12; i++)
				{
					if (buf[i] == 0xAF && buf[i + 1] == 0xFE && buf[i + 2] == 0xC0 && buf[i + 3] == 0xC2)
					{
						cout << "MessageSize :" << MessageSize << endl;
						if (data_len - i >= MessageSize + HEAD_LEN)
						{
							RecePro(i, MessageSize + HEAD_LEN);
							i += MessageSize + HEAD_LEN - 1;
						}
						else
							break;
					}
				}
				if (i > 0)
				{
					if (i < data_len)
					{
						for (int j = 0; j < data_len - i; j++)
							buf[j] = buf[i + j];
						data_len -= i;
					}
					else
						data_len = 0;
				}
			}

			void RecePro(int start_pos, int len)
			{
				// cout << "RecePro Begin" << endl;
				int DataType = buf[start_pos + 14] << 8 | buf[start_pos + 15];
				// cout << "MessageSize :" << MessageSize << endl;
				if (DataType == 0x2202)
				{
					cout << "DecodeRaw" << endl;
					DecodeRaw(start_pos, len);
				}
				else if (DataType == 0x2221)
				{
					// cout << "DecodeObject" << endl;
				}
				else
				{
					// printf("Error DataType: %04x \n", DataType);
				}
			}

			//for deteting rain
			void accumulatePtType(LPoint ptType, double ptDist)
			{
						}	

			void DecodeRaw(int start_pos, int len)
			{
				int i;
				uint64_t tm = 0;
				for (int i = 0; i < 8; i++)
				{
					tm <<= 8;
					tm |= buf[start_pos + 16 + i];
				}
				LRRaw lrw;
				lrw.tm = tm;
				int ix = start_pos + HEAD_LEN + 22;
				lrw.atpr = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.atpr);
				lrw.sa = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.sa);
				lrw.ea = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.ea);
				//lrw.npt = ntohs(*((short*)&(buf[ix])));
				lrw.npt = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.npt);
				lrw.yaw = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.yaw);
				lrw.pitch = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.pitch);
				lrw.roll = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.roll);
				lrw.mpx = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.mpx);
				lrw.mpy = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.mpy);
				lrw.mpz = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.mpz);
				lrw.flag = buf[ix] | (buf[ix + 1] << 8);
				ix += sizeof(lrw.flag);
				// ofs<<"yaw: "<<lrw.yaw<<" pitch: "<<lrw.pitch<<" roll: "<<lrw.roll<<" mx: "<<lrw.mpx<<" my: "<<lrw.mpy<<" mz: "<<lrw.mpz<<endl;
				if ((len - ix + start_pos) != lrw.npt * 10)
				{
					//if (filestream)
					//{
					//	fwrite("not enough point", sizeof(char), 512, filestream);
					//}
					//{
					//	cout << "not enough point,need:" << lrw.npt * 10 << " have:" << (len - ix) + start_pos << endl;
					//}
					return;
				}
				// cout<< (int)(lrw.flag) << " " << (int)(lrw.flag & 0x0400) << " " << (lrw.flag & 0x0400 == 0) <<endl;
				//Lux8
				if ((int)(lrw.flag & 0x0400) == 0)//Bit 10:(0 = front = mirror facing down, 1 = rear = mirror facing up)
				{
					cout<<"facing down"<<endl;
					int num1 = 0, num2 = 0;
					for (i = 0; i < lrw.npt; i++)
					{
						LPoint lpt;
						lpt.ly = (byte)(buf[ix] & 0x0F);//取低4位as layer id{0~3}
					// if (lrw.flag >= 0x0400)
					// {
					// 	lpt.ly = (byte)(lpt.ly|= 0x04);//mirror facing up, change layer id{4~7}
					// }
					// cout<<(int)lpt.ly<<"";
					lpt.echo = (byte)(buf[ix] & 0xF0);//取高4位as echo id
					ix += 1;
					lpt.flag = buf[ix];
					ix += 1;
					lpt.ha = buf[ix] | (buf[ix + 1] << 8);
					ix += sizeof(lpt.ha);
					lpt.rd = buf[ix] | (buf[ix + 1] << 8);
					ix += sizeof(lpt.rd);
					lpt.epw = buf[ix] | (buf[ix + 1] << 8);
					ix += sizeof(lpt.epw);
					lpt.rev = buf[ix] | (buf[ix + 1] << 8);
					ix += sizeof(lpt.rev);

					lpt.a = lpt.ha * 2 * TiEV_PI / lrw.atpr;
					lpt.d = (double)lpt.rd / 100;

					lpt.x = lpt.d * sin(-lpt.a);
					lpt.y = lpt.d * cos(lpt.a);

					//check point type
					if ( !MATCH(lpt.flag, FLAG_CLUTTER) && \
							!MATCH(lpt.flag, FLAG_GROUND) && \
							!MATCH(lpt.flag, FLAG_DIRT) && \
							lpt.d > 1 )//points not in  clutter(atmospheric)
					{
						num_obstacle_pts++;
						lrw.lpts.push_back(lpt);
					}
					else if (MATCH(lpt.flag, FLAG_CLUTTER)) 
					{
						num_clutter_pts++;
					}
					else if (MATCH(lpt.flag, FLAG_DIRT)) 
					{
						num_dirt_pts++;
					}
				}
				else if (lrw.flag & 0x0400 != 0)
				{
					cout<<"facing up"<<endl;
					int num1 = 0, num2 = 0;
					for (i = 0; i < lrw.npt; i++)
					{
						LPoint lpt;
						lpt.ly = (byte)(buf[ix] & 0x0F);
						lpt.ly = (byte)(lpt.ly|= 0x04);
						lpt.echo = (byte)(buf[ix] & 0xF0);
						ix += 1;
						lpt.flag = buf[ix];
						ix += 1;
						lpt.ha = buf[ix] | (buf[ix + 1] << 8);
						ix += sizeof(lpt.ha);
						lpt.rd = buf[ix] | (buf[ix + 1] << 8);
						ix += sizeof(lpt.rd);
						lpt.epw = buf[ix] | (buf[ix + 1] << 8);
						ix += sizeof(lpt.epw);
						lpt.rev = buf[ix] | (buf[ix + 1] << 8);
						ix += sizeof(lpt.rev);

						lpt.a = lpt.ha * 2 * TiEV_PI / lrw.atpr;
						lpt.d = (double)lpt.rd / 100;

						lpt.x = lpt.d * sin(-lpt.a);
						lpt.y = lpt.d * cos(lpt.a);//??? - fabs(lpt.x * 0.05)
						//
						//check point type
						if ( !MATCH(lpt.flag, FLAG_CLUTTER) && \
								!MATCH(lpt.flag, FLAG_GROUND) && \
								!MATCH(lpt.flag, FLAG_DIRT) && \
								lpt.d > 1 )//points not in  clutter(atmospheric)
						{
							num_obstacle_pts++;
							lrw.lpts.push_back(lpt);
						}
						else if (MATCH(lpt.flag, FLAG_CLUTTER)) 
						{
							num_clutter_pts++;
						}
						else if (MATCH(lpt.flag, FLAG_DIRT)) 
						{
							num_dirt_pts++;
						}
					}
				}
				
				if((float)num_clutter_pts/(float)num_obstacle_pts > TOL_RAIN_RATIO)
				{
					DecodeRainSignal(true);
				}
				else
				{
					DecodeRainSignal(false);
				}

				if (lrw.lpts.size() > 0)
				{
					DecodeMatrix(lrw);
				}
			}

			const int WIDTH = TiEV::GRID_COL;
			const int HEIGHT = TiEV::GRID_ROW;
			const int CENTER_X = TiEV::CAR_CEN_ROW;
			const int CENTER_Y = TiEV::CAR_CEN_COL;
			const int OFFSET_X = -12;
			const int OFFSET_Y = 0;
			const int BASE = 1; //for display brighter in imshow
			//const unsigned char SICK =  1 + BASE, LUX4 = 1 + BASE, LUX8 = 1 + BASE, HDL64 = 1 + BASE, BLOCK = 2 + BASE;

			void DecodeRainSignal(bool isRain)
			{
				MsgRainDetectionSignal msg;
				msg.timestamp = TiEV::getTimeStamp();
				msg.rain_signal = isRain;
				myzcm_udpm.publish("MsgRainDetectionSignal", &msg);
			}

			void DecodeMatrix(LRRaw lrw)
			{
				//lcm
				mapData.resolution = TiEV::GRID_RESOLUTION;
				mapData.rows = HEIGHT;
				mapData.cols = WIDTH;
				mapData.center_col = WIDTH / 2;
				mapData.center_row = CENTER_X;
				//
				int i;
				objmid = 1000;
				int errorpoi = 0; 
				for (i = 0; i < lrw.npt; i++)
				{
					int row = CENTER_X - (int)(lrw.lpts[i].y * 5) + OFFSET_X;
					if(lrw.lpts[i].ly < 0 || lrw.lpts[i].ly > 7)
					{
						errorpoi++;
						continue;
					}
					//if (
					//       	      lrw.lpts[i].ly == 0 && row < 154//154
					//	|| lrw.lpts[i].ly == 1 && row < 175//175
					// 	|| lrw.lpts[i].ly == 2 && row < 189//189
					// 	|| lrw.lpts[i].ly == 3 && row < 205//205
					// 	|| lrw.lpts[i].ly == 4 && row < 154//154
					// 	|| lrw.lpts[i].ly == 5 && row < 175//175
					// 	|| lrw.lpts[i].ly == 6 && row < 189//189
					// 	|| lrw.lpts[i].ly == 7 && row < 205//205
					// 	)
					// 	continue; 
					// cout<<(int)lrw.lpts[i].ly<<"";
					int col = (int)(lrw.lpts[i].x * 5 + CENTER_Y) + OFFSET_Y;
					int echo = lrw.lpts[i].echo >> 4;
					if (row >= 0 && row < HEIGHT && col >= 0 && col < WIDTH)// && (lrw.lpts[i].ly == 0 || lrw.lpts[i].ly == 4)) //&& echo == 0)
					{
						mapData.cells[row][col] = 1;
					}

				}
				//manually remove the outlier
				cout << "errorpoints num: " <<errorpoi<< endl;
				//mapData.cells[288][75] = 0;
				mapData.cells[TiEV::CAR_CEN_ROW - 12][TiEV::CAR_CEN_COL] = 0;
				// cout << "Send Message!" << endl;
				myzcm_ipc.publish("LUXMAP", &mapData);
				memset(mapData.cells,0,sizeof(mapData.cells));
			}

			void closeall()
			{
				for (int i = 0; i < LISTEN_NUM; i++)
				{
					if (fd_a[i] != 0)
						close(fd_a[i]);
				}
			}
	};
}
#endif /* Header_h */
