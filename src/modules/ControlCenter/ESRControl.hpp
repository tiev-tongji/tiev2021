#ifndef _ESRCONTROL_
#define _ESRCONTROL_

#include "iostream"
#include <fstream>
#include "string"
#include "math.h"
#include <vector>
#include <stdio.h>
#include <list>
#include <unistd.h>
#include <asm/types.h>
#include <mutex>
#include <thread>
#include <stdlib.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iomanip>
#include "ControlCenterCommon.h"
#include "nature.h"
#include "structESROBJINFO.hpp"

#define SpdN 10
#define NUM_OBJ 64
#define FILTER 100
#define FILTER_ASSOCIATION 25
#define MsgHeapSize 128
#define TOL 0.2


struct InPathID
{
	int path_id_ACC_stat;	//In-path ACC target ID (stationary or oncoming)
	int path_id_ACC;		//In-path ACC target ID (moving or moveable)
	int path_id_CMBB_move;
	int path_id_CMBB_stat;
	int path_id_FCW_stat;  //In-path moving FCW target ID
	int path_id_FCW_move;  //In-path stationary FCW target ID
};


struct VehicleInfo
{
	double 			speed; 	//scale 0.0625
	unsigned char 	speed_direction;//0 forward,1 reverse
	unsigned char   speed_validity; //0 no vehicle speed or invalid 1 vehicle speed avaiable and valid

	double 			yaw_rate;		//deg/s scale 0.0625   + clockwise
	unsigned char  	yaw_rate_validity;//0

	double 			steering_angle;// deg 0 steering wheel centered
	unsigned char 	steering_angle_sign;//0 velocity left (counterlockwise) 1 velocity right (clockwise)

	double 			steering_angle_rate; //	deg/s  scale 0.0625
	unsigned char 	steering_angle_rate_sign; //0 left turn 1 right turn

	unsigned char 	steering_angle_validity;//0 no steering angle sensor or invalid, 1 steering angle sensor data available and valid
	double 			radius_curvature;
};


struct CarsInfo
{
	double 			range;  //m scale 0.1 0~204.7
	double 			velocity;//m/s  scale 0.01  -81.92~+81.91 + away from sensor
	double 			azimuth; // deg -51.2~ 51.1 + clockwise
	double 			width;  //0.5 scale
	unsigned char 	isMoving; //0 stationary 1 moving
	double 			x;
	double 			y;
};


struct objInfo{
	double speedH;
	double speedV;
	double objH;
	double objV;
	float objWidth;
};

// struct objInfoInt{
// 	double speedH;
// 	double speedV;
// 	int objH;
// 	int objV;
// };


class ESRControl{
public:
    ESRControl();
    ESRControl(const int canPort);
    ~ESRControl();
    void init();
    void setNavInfo(nav_info_t& navInfo);
    //structESRMAP* getEsrMapPtr();
    structESROBJINFO* getEsrObjInfoPtr();
	//void esrMapLock(){esr_map_lock.lock();}
	void esrObjLock(){esr_objinfo_lock.lock();}
	//void esrMapUnLock(){esr_map_lock.unlock();}
	void esrObjUnLock(){esr_Objinfo_lock.unlock();}
private:
    void canInfoRead();
    void canInfoWrite();
    void startEsr();
    int work(int num);
	void inPath_ID_ACC_FCW(can_frame *Message);

private:
    int EHB_CAN_PORT;
    nav_info_t navInfo;
    std::mutex nav_info_lock;
    //std::mutex esr_map_lock;
    std::mutex esr_objinfo_lock;
    std::mutex stLstTop_mutex;
    //structESRMAP* myEsrMap;
    structESROBJINFO* myEsrObjInfo;
    int stLstTop;
    unsigned char TrackInfo[MsgHeapSize][64][8];
    unsigned char Tracks[70];
    InPathID ACC_FCW;
    int objWeights[NUM_OBJ];
    objInfo objArr[NUM_OBJ];
    int fnum;
    std::list<objInfo> oldObj;
};

#endif
