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
 * @LastEditTime: 2019-10-25 18:55:42
 */

#include "RemoteControl.hpp"

remoteControl::remoteControl(){

}

remoteControl::~remoteControl(){

}

int remoteControl::GPIO_Init(){
	int status = system("echo 393 > /sys/class/gpio/export");
	if(status < 0){
		cout << "Init GPIO Error" << endl;
		return -1;
	}
	status = system("echo in > /sys/class/gpio/gpio393/direction");
	if(status < 0){
		cout << "Init GPIO DIR Error" << endl;
		return -1;
	}
	return 1;
}

int remoteControl::GetGPIOValue(){
	FILE* pp = popen("cat /sys/class/gpio/gpio393/value", "r");
	if( !pp ){
		cout << "Read GPIO393 Value Error!" << endl;
		pclose(pp);
		return -1;
	}
	char buf[10];
	while(fread(buf, 10, sizeof(buf), pp) > 0);
	pclose(pp);
	if(buf[0] == '0'){
		return 0;
	}
	else if(buf[0] == '1'){
		return 1;
	}
	else{
		cout << "Error: " << buf << endl;
		return -1;
	}
}
