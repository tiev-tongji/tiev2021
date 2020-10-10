/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: ControlCenterCommon.h
 * @Descripttion: 控制中枢的通用文件，定义基本结构体与类型等信息
 * @Author: Ding Yongchao
 * @version: 
 * @Date: 2019-10-25 18:48:52
 * @FunctionList: 
 * @History: 
 * @LastEditors: Ding Yongchao
 * @LastEditTime: 2019-10-25 22:23:49
 */
#ifndef CONTROL_CENTER_COMMON_H_
#define CONTROL_CENTER_COMMON_H_

#include <string.h>
#include <string>
#include <fstream>

#include "sys_logger.h"

typedef int8_t STATE;

const int8_t CC_OK = 0;
const int8_t CC_ERR = -1;

// 调试位
const bool debug_flag = false;
//const bool debug_flag = true;

#define DEBUG(info) if(debug_flag == true){ \
						std::cout << info << std::endl;}
#define INFO(info) std::cout << info << std::endl;
#define ERR(info) std::cerr << info << std::endl;

typedef struct veh_info{
	float speed;
	float angle;
	std::string timestamp;
} veh_info_t;

typedef struct control_params{
	float acc_P = 0;
	float acc_I = 0;
	float acc_D = 0;
	
	float break_P = 0;
	float break_I = 0;
	float break_D = 0;

	float steer_P = 0;
	float steer_I = 0;
	float steer_D = 0;
} control_params_t;

typedef struct nav_info{
	double mHeading;
	double utmX, utmY;
	double speed, yawRate;
	std::string timestamp;
} nav_info_t;

inline void assign_params(std::string& name, float& num, std::string param_name, float* assign_num){
	if (name == param_name){
		*assign_num = num;
		DEBUG(param_name << ": " << *assign_num);
	}
	return;
}

inline STATE load_params_file(const std::string& params_file, control_params_t* params){
	std::ifstream fin(params_file.c_str(), std::ios::in);
	if (!fin){
		ERR("open file " << params_file << " error");
		return CC_ERR;
	}
	std::string name;
	float number;
	while (fin >> name >> number){
		assign_params(name, number, "acc_P", &params->acc_P);
		assign_params(name, number, "acc_I", &params->acc_I);
		assign_params(name, number, "acc_D", &params->acc_D);
		assign_params(name, number, "break_P", &params->break_P);
		assign_params(name, number, "break_I", &params->break_I);
		assign_params(name, number, "break_D", &params->break_D);
		assign_params(name, number, "steer_P", &params->steer_P);
		assign_params(name, number, "steer_I", &params->steer_I);
		assign_params(name, number, "steer_D", &params->steer_D);
	}

	return CC_OK;
}

#endif // CONTROL_CENTER_COMMON_H_
