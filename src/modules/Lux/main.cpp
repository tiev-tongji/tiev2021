/*
 * Copyright (c) 2020 TiEV (Tongji Intelligent Electric Vehicle).
 * 
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 * 
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
 //
//  main.cpp
//  lux
//
//  Created by 高昕宇 on 2017/3/26, modified by 吴岩 管林挺 赵君峤 on 2020/10/20.
//  Copyright © 2020年. All rights reserved.
//

#include <iostream>
#include <thread>
#include <mutex>
#include "lux_8.h"

namespace TiEV
{
	void Handler::handleNAVINFOMessage(const zcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const structNAVINFO* msg)
	{
		int i;
		printf("Received message on channel \"%s\":\n", chan.c_str());
		pos_mutex.lock();
		memcpy(&currentPose, msg, sizeof(structNAVINFO));
		pos_mutex.unlock();
	}

	void zcm_func()
	{
		if (!myzcm_udpm.good())
			return;
		Handler handlerObject;
		myzcm_udpm.subscribe("NAVINFO", &Handler::handleNAVINFOMessage, &handlerObject);
		while (0 == myzcm_udpm.handle());
	}
}

using namespace TiEV;

int main(int argc, const char * argv[])
{
	// insert code here...
	//lcm
	std::thread Zcm_thread(zcm_func);
	Zcm_thread.detach();

	//Lux
	Laser_8 lux;
	lux.init_client("192.168.0.10", 12002);
	lux.start_recieve_client();
	lux.closeall();

	return 0;
}
