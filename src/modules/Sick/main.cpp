/*
 * Copyright (c) 2018 TiEV (Tongji Intelligent Electric Vehicle).
 * 
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 * 
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
//
//  main.cpp
//  sick
//
//  Created by 高昕宇 on 2017/9/28. modified by 吴岩 赵君峤 on 2017/10/20.
//  Copyright © 2017年 Rizia. All rights reserved.
//

#include <iostream>
#include <thread>
#include <mutex>
#include "sick.h"

namespace TiEV
{
	void handleNAVINFOMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structNAVINFO *msg)
	{
		int i;
		printf("Received message on channel \"%s\":\n", chan.c_str());
		pos_mutex.lock();
		memcpy(&currentPose, msg, sizeof(structNAVINFO));
		pos_mutex.unlock();
	}

	void zcm_func()
	{
		if (!myzcm.good())
			return;
		Handler handlerObject;
		myzcm.subscribe("NAVINFO", &Handler::handleNAVINFOMessage, &handlerObject);
		myzcm.run();
	}
} // namespace TiEV

using namespace TiEV;

int main(int argc, const char *argv[])
{
	// insert code here...
	//zcm
	std::thread Zcm_thread(zcm_func);
	Zcm_thread.detach();

	//Sick
	Sick sick_front, sick_back;
	sick_front.init_client("192.168.222.22", 2112); //
	sick_back.init_client("192.168.222.11", 2112);

	while (1)
	{
		sickmap_init();
		//kind = 0:front雷达代码解析，king=1:back雷达代码解析
		sick_front.start_recieve_client(0);
		sick_back.start_recieve_client(1);
		myzcm.publish("SICKMAP", &mapData);
		memset(mapData.cells, 0, sizeof(mapData.cells));
	}
	std::cout << "Hello, World!\n";

	return 0;
}
