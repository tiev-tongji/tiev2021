/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 18:48:52
 * @FunctionList: 
 * @History: 
 * @LastEditors: Huang Tengfei
 * @LastEditTime: 2022-02-28 18:55:25
 */

/* Frame format:
		| 帧头 | 地址1 | 地址2 | 按键值 | 振荡参数 | 帧尾 |
 A: | 01 | 3E | E7 | 2B | 15 | 00 |
 B:	| 01 | 3E | E7 | 65 | 05 | 00 |
*/
#ifndef REMOTECONTROL_HPP
#define REMOTECONTROL_HPP


#include "SerialPort.hpp"

#include <unistd.h> 
#include <stdio.h>
#include <thread>


class RemoteController {
public:
	RemoteController() {
		is_brake = false;
		usb_port.SetDevice("/dev/remote_control");
		usb_port.SetBaudRate(CppLinuxSerial::BaudRate::B_9600);
	}
	~RemoteController(){stop();}

	void startReceiving() {
		try {
			usb_port.Open();
		} catch(CppLinuxSerial::Exception& excpt) {
			std::cout<<excpt.what()<<std::endl;
			std::cout<<"Check device is pluged in, or try to run change_device_authority.sh." << std::endl;
			return;
		}
		receiving_ = true;
		rcv_thread = std::thread(&RemoteController::Receive, this);
	}

	void stop() {
		if(receiving_) {
			receiving_ = false;
			rcv_thread.join();
			usb_port.Close();
		}
	}

	bool isBrake() { return is_brake; }

private:
	struct FRAME {
		enum : unsigned char {
			HEAD 	= 0x01,
			TAIL 	= 0x00,
			ADDR1 	= 0x3E,
			ADDR2 	= 0xE7,
			ENABLE 	= 0x15,
			DISABLE = 0x05
		};
	};

	void Receive() {
		while (receiving_) {
			usb_port.Read(buffers);
			for (size_t i = 0; i < buffers.size()-5; i++) {
				if (buffers[i] == FRAME::HEAD && buffers[i+5] == FRAME::TAIL)
					HandleRemoteControlFrame(buffers.substr(i, 6).c_str());
				i += 6;
			}
			usleep(50*1000);
		}
	}

	void HandleRemoteControlFrame(const char* frame) {
		if (frame[1] != FRAME::ADDR1 || frame[2] != FRAME::ADDR2)
			return;

		unsigned char button_pressed = frame[4];
		if (button_pressed == FRAME::ENABLE) 
			is_brake == true;

		else if (button_pressed == FRAME::DISABLE) 
			is_brake == false;

		// std::cout<<"Send remote_control_enable = "<<(remote_control_msg.enable ? "true" : "false")<<std::endl;
	}

private:
	bool is_brake;
	bool receiving_;
	CppLinuxSerial::SerialPort usb_port;
	std::string buffers;
	std::thread rcv_thread;
};

#endif