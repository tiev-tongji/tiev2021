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
 * @LastEditTime: 2019-10-25 18:55:25
 */
#ifndef REMOTECONTROL_HPP
#define REMOTECONTROL_HPP

// TODO: 考虑到设备的稳定性，及其代码的鲁棒性
// TODO: 暂时不使用该功能，交付给单片机进行处理使用

class remoteControl{
	remoteControl();
	~remoteControl();
	int GPIO_Init();
	int GetGPIOValue();
};

#endif