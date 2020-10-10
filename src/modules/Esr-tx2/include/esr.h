#include "iostream"
#include <fstream>
#include "string"
#include "math.h"
#include <vector>
#include <stdio.h>
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

#include <zcm/zcm-cpp.hpp>
#include "msg/include/structESRMAP.hpp"
#include "msg/include/structNAVINFO.hpp"
#include "common/nature.h"


class Handler {
public:
    ~Handler(){};

    structNAVINFO positionStamp;
    std::mutex navinfoMutex;

    std::mutex navinfoMutex,carStatusMutex;

    void handleNAVINFOMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structNAVINFO *msg)
    {
    	int i;
    	// printf("Received message on channel \"%s\":\n", chan.c_str());
    	navinfoMutex.lock();
    	memcpy(&positionStamp, msg, sizeof(structNAVINFO));
    	navinfoMutex.unlock();
	}
};



struct InPathID
{
	bool path_id_ACC_stat;	//In-path ACC target ID (stationary or oncoming)
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


class Esr{

public:
	Esr();
	~Esr(){};
	int C1;
	// TPCANMsg Message;
	// TPCANStatus Status;

	void inPath_ID_ACC_FCW(can_frame *Message);
	void canInfoRead();
	void canInfoWrite();

private:

	InPathID  inpathid;
	VehicleInfo vehicle;
	CarsInfo carObj[64];
};

