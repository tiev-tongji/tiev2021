#include <list>
#include <cstring>
#include "esr.h"

using namespace std;

//zcm
zcm::ZCM             myzcmSub{"ipc"};//todo
zcm::ZCM             myzcmPub{"udpm://239.255.76.67:7667?ttl=1"};
structESRMAP         myEsrMap;
Handler              myhandler;

struct objInfo{
	double speedH;
	double speedV;
	double objH;
	double objV;
};

struct objInfoInt{
	double speedH;
	double speedV;
	int objH;
	int objV;
};

list<objInfo> oldObj;

#define MapL 401
#define MapW 151
#define OriginX 75
#define OriginY 301
#define PerGrid 0.2
#define SpdN 10
#define NUM_OBJ 64
#define FILTER 100
#define MsgHeapSize 128
#define TOL 0.2

#define  PCAN_DEVICE1 PCAN_PCIBUS2//PCAN_USBBUS1

int stLstTop = 0;

mutex  stLstTop_mutex;

unsigned char TrackInfo[MsgHeapSize][64][8];

unsigned char Tracks[70];//obj movable 64 objs

InPathID ACC_FCW;

int objWeights[NUM_OBJ];

objInfo objArr[NUM_OBJ];

// msg_esrObj_SPS esrObjInfo;
int fnum = 0;

// Mat mapEsr(MapL, MapW, CV_8U, Scalar(0));
//int fnum = 0;

int work(int num){
	int ObjectID;
	int angle, range, width, rangespd, latspd, bridge;
	double angleR, rangeR, widthR, rangespdR, latspdR;
	double objX, objY, objB, objE;
	// mapEsr = Mat(MapL, MapW, CV_8U, Scalar(0));
	int numobj = 0;
	for (list<objInfo>::iterator i = oldObj.begin(); i != oldObj.end(); i++)
	{

		i->objV -= (i->speedV * 0.05) / PerGrid;
		i->objH -= (i->speedH * 0.05) / PerGrid;
		if (i->objV < 0 || i->objV > MapL || i->objH < 0 || i->objH > MapW)
		{
			i = oldObj.erase(i);
			cout << "out of range" << endl;
		}

	}
	for (ObjectID = 0; ObjectID < NUM_OBJ; ObjectID++){

//        if (ObjectID == ACC_FCW.path_id_CMBB_stat ||
//            ObjectID == ACC_FCW.path_id_CMBB_move ||
//            ObjectID == ACC_FCW.path_id_FCW_stat ||
//            ObjectID == ACC_FCW.path_id_FCW_move ||
//            ObjectID == ACC_FCW.path_id_ACC ||
//            ObjectID == ACC_FCW.path_id_ACC_stat)
            // cout<<"Tracks[ObjectID] & 0x20: "<<((Tracks[ObjectID] & 0x20)>>5)<<endl;
			//if ((((Tracks[ObjectID] & 0x20)>>5) != 0 && (int)(Tracks[ObjectID] & 0x80) != 0))
		// || objWeights[ObjectID] > 0){
        if (((Tracks[ObjectID] & 0x20)>>5) != 0)//is dynamic obj
        {
            if (TrackInfo[messgeNum][ObjectID][0] == 0
                && TrackInfo[messgeNum][ObjectID][1] == 0
                && TrackInfo[messgeNum][ObjectID][2] == 0
                && TrackInfo[messgeNum][ObjectID][3] == 0)
                continue; //no data


            cout << "------------------------------------" << endl;
            //if (((int)(Tracks[ObjectID] & 0x20) != 0 && (int)(Tracks[ObjectID] & 0x80) != 0)) {
            cout << ObjectID << " SEE A DYNAMIC OBJ!!!!!!!" << endl;
            objWeights[ObjectID] = FILTER;

            latspd = ((TrackInfo[messgeNum][ObjectID][0] & 0xFC) >> 2); //横向速度?
            angle = ((TrackInfo[messgeNum][ObjectID][1] & 0x1F) << 5) + ((TrackInfo[messgeNum][ObjectID][2] & 0xF8) >> 3);
            range = ((TrackInfo[messgeNum][ObjectID][2] & 0x07) << 8) + TrackInfo[messgeNum][ObjectID][3];//
            width = ((TrackInfo[messgeNum][ObjectID][4] & 0x3C) >> 2);
            bridge = ((TrackInfo[messgeNum][ObjectID][4] & 0x80) >> 7);
            rangespd = ((TrackInfo[messgeNum][ObjectID][6] & 0x3F) << 8) + TrackInfo[messgeNum][ObjectID][7];

            //	cout << "fast moving ID:  " << ObjectID << endl;
            if (angle < (1 << 9))
                angleR = angle * 0.1; //angle > 0
            else
                angleR = (angle - (1 << 10)) * 0.1;//angle < 0

            angleR = angleR * 3.14159 / 180.0;    //convert to rad
            rangeR = range * 0.1;        //get the value of obj's range
            widthR = width * 0.5;        //get the value of obj's width

            if (rangespd < (1 << 13))
                rangespdR = rangespd * 0.01; //v > 0
            else
                rangespdR = (rangespd - (1 << 14)) * 0.01;//v < 0
            //
            if (latspd < (1 << 5))
                latspdR = latspd * 0.25;
            else
                latspdR = (latspd - (1 << 6)) * 0.25;


            cout << "rangespdR: " << rangespdR << " latspdR: " << latspdR << endl;//get the value of two speed

            //todo
            double rangespd_h = rangespdR * sin(angleR) + latspdR * cos(angleR);//get the horizontal speed
            double rangespd_v = rangespdR * cos(angleR) + latspdR * sin(angleR);//get the vertical speed
            objX = OriginX + rangeR * sin(angleR) / PerGrid;//get the X position of obj in gridmap
            objY = OriginY - rangeR * cos(angleR) / PerGrid;//get the Y position of obj in gridmap

            if (sqrt((objArr[ObjectID].objH - objX) * (objArr[ObjectID].objH - objX) +
                     (objArr[ObjectID].objV - objY) * (objArr[ObjectID].objV - objY))
                > 25) //this obj distance moved from last frame to current frame > 25
            {
            		//	cout << "left id:" << ObjectID << " speedH:" << objArr[ObjectID].speedH << " speedV:" << objArr[ObjectID].speedV << endl;
            	if (abs(objArr[ObjectID].speedV) > TOL || abs(objArr[ObjectID].speedH) > TOL)
            	{
            		oldObj.push_back(objArr[ObjectID]);
            	}
                objArr[ObjectID].objH = objX - 4;//update the position of current obj in gridmap but why x - 4 and y - 15
                objArr[ObjectID].objV = objY - 15;
            } else //distance moved < 25
            {
                objArr[ObjectID].objH = 0.5 * (objX - 4) + 0.5 * objArr[ObjectID].objH;//middle value?
                objArr[ObjectID].objV = 0.5 * (objY - 15) + 0.5 * objArr[ObjectID].objV;
            }
            ++numobj;

            //todo
            objX = objArr[ObjectID].objH;
            objY = objArr[ObjectID].objV;
            if (objY < 0) continue;  //y top not in map
            objB = objX - (widthR * 0.5 / PerGrid);
            if (objB < 0) continue;  //x left not in map
            objE = objX + (widthR * 0.5 / PerGrid);
            if (objE > MapW) continue;//x right not in map



            // cout << ObjectID << " SEE A DYNAMIC OBJ!!!!!!!" << endl;
            cout << "objV: " << objArr[ObjectID].objV << " objH: " << objArr[ObjectID].objH << endl;
            objArr[ObjectID].speedH = rangespd_h;
            objArr[ObjectID].speedV = rangespd_v;
            cout << "speedV: " << objArr[ObjectID].speedV << ' ' << "speedH: " << objArr[ObjectID].speedH << endl;
            cout << "width: " << widthR * 0.5 << endl;
            cout << "ismove: " << ((Tracks[ObjectID] & 0x20) >> 5) << endl;// 1
            cout << "current numobj: " << numobj << endl;
            //for (int j = 0; j <= abs((speed-rangespdR)/ SpdN) + 1; j++)

            for (int j = 0; j <= 11; j++)//height suppose to be 12
            {
                for (int i = (int) objB; i <= (int) objE + 10; i++)//width suppose to be (widthR + 10)
                {
                	myEsrMap.cells[(int) objY + j][i] = 255;
                    // mapEsr.ptr<uchar>((int) objY + j)[i] = 255/*((int(widthR / PerGrid) == 0) ? 100 : 240)*/;
                }
            }
        }

        else
        {
        	cout << ObjectID << " NOT SEE  Weight: " << objWeights[ObjectID] << endl;
        	cout << "speedV: " << objArr[ObjectID].speedV << ' ' << "speedH: " << objArr[ObjectID].speedH << endl;
        	objWeights[ObjectID]--;
        	objArr[ObjectID].objV -= (objArr[ObjectID].speedV * 0.05) / PerGrid;
			//objArr[ObjectID].objH -= (objArr[ObjectID].speedH * 0.05) / PerGrid;
        	cout << "objV: " << objArr[ObjectID].objV << " objH: " << objArr[ObjectID].objH << endl;
        	if (abs(objArr[ObjectID].speedV) < TOL && abs(objArr[ObjectID].speedH) < TOL)
        	{
        		objWeights[ObjectID] = 0;
        		cout << "speed 0" << endl;
        		continue;
        	}
        	if (objArr[ObjectID].objV < 0 || objArr[ObjectID].objV > MapL || objArr[ObjectID].objH < 0 || objArr[ObjectID].objH > MapW)
        	{
        		objWeights[ObjectID] = 0;
        		cout << "out of range" << endl;
        		continue;
        	}
        	else
        	{
        		objX = objArr[ObjectID].objH;
        		objY = objArr[ObjectID].objV;
        		if (objY < 0) continue;
        		objB = objX - (widthR*0.5 / PerGrid);
        		if (objB < 0) continue;
        		objE = objX + (widthR*0.5 / PerGrid);
        		if (objE > MapW) continue;

				//for (int j = 0; j <= abs((speed-rangespdR)/ SpdN) + 1; j++)
        		for (int j = 0; j <= 5; j++)
        			for (int i = (int)objB; i <= (int)objE + 4; i++)
        				myEsrMap.cells[(int) objY + j][i] = ((int(widthR / PerGrid) == 0) ? 100 : 240);
        				// mapEsr.ptr<uchar>((int)objY + j)[i] = ((int(widthR / PerGrid) == 0) ? 100 : 240);
        		
        	}
        }
    }
    
    //draw car
    // for(int i = 68; i<= 82;i++){
    // 	myEsrMap.cells[300][i] = 255;
    // 	myEsrMap.cells[320][i] = 255;
    // 	// mapEsr.ptr<uchar>(300)[i] = 255;
    // 	// mapEsr.ptr<uchar>(320)[i] = 255;
    // }
    // for(int j = 300;j<=320;j++){
    // 	myEsrMap.cells[j][68] = 255;
    // 	myEsrMap.cells[j][82] = 255;
    // 	// mapEsr.ptr<uchar>(j)[68] = 255;
    // 	// mapEsr.ptr<uchar>(j)[82] = 255;
    // }
    // 

    //outputMap(); //all the 64 obj can view in the map
    int objNum = 0; //every time start new counts form 0, so send the current newest objs???

    myhandler.navinfoMutex.lock();
    structNAVINFO navinfo = myhandler.positionStamp;
    myhandler.navinfoMutex.unlock();

    myEsrMap.timestamp = TiEV::getTimeStamp();
    myEsrMap.mHeading = navinfo.mHeading;//position and timestamp info
    myEsrMap.utmX = navinfo.utmX;
    myEsrMap.utmY = navinfo.utmY;
    myEsrMap.resolution = TiEV::GRID_RESOLUTION;
    myEsrMap.rows = TiEV::GRID_ROW;
    myEsrMap.cols = TiEV::GRID_COL;
    myEsrMap.center_col = TiEV::CAR_CEN_COL;
    myEsrMap.center_row = TiEV::CAR_CEN_ROW;

    int count = 0;//esrObjInfo.Count;//current obj counts

    for (size_t i = 0; i < NUM_OBJ; i++) //current 64 obj info save to esrObjInfo and objArrSel
    {
        if (objWeights[i] > 0)//is dynamic obj
        {

            // esrObjInfo.AllObjs[count].Id = i;//obj info
            // esrObjInfo.AllObjs[count].V = 0;//todo speedH or speedV
            // esrObjInfo.AllObjs[count].X = objArr[i].objH;
            // esrObjInfo.AllObjs[count].Y = objArr[i].objV;
            // esrObjInfo.Count = ++count;

            if (++objNum == 64) break;
        }
    }

    if (objNum > 64) {
        cout << "ERROR!!!!!  more than 64 dynamic object???????" << endl;
    }
    myzcmPub.publish("ESRMAP", &myEsrMap);
    memset(&myEsrMap, 0, sizeof(myEsrMap));
    usleep(10);
    return 0;
}


Esr::Esr(){


	struct sockaddr_can addr_can1;
	struct ifreq ifr_can1;
	C1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	strcpy(ifr_can1.ifr_name, "can1");
	ioctl(C1, SIOCGIFINDEX, &ifr_can1);
	addr_can1.can_family = AF_CAN;
	addr_can1.can_ifindex = ifr_can1.ifr_ifindex;

	bind(C1, (struct sockaddr *)&addr_can1, sizeof(addr_can1));

	struct can_filter rfilter_can1[67];
	rfilter_can1[0].can_id = 0x4E0;            //initial msg ID
    rfilter_can1[0].can_mask = CAN_SFF_MASK;
    rfilter_can1[1].can_id = 0x4E3;
    rfilter_can1[1].can_mask = CAN_SFF_MASK;
    rfilter_can1[2].can_id = 0x540;            
    rfilter_can1[2].can_mask = CAN_SFF_MASK;

    for(int i = 0;i < 64; i++){
    	rfilter_can1[3 + i].can_id = 0x500 + i;
    	rfilter_can1[3 + i].can_mask = CAN_SFF_MASK;
    }
    setsockopt(C1, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter_can1, sizeof(rfilter_can1));	
}


void Esr::canInfoWrite() {

    myhandler.navinfoMutex.lock();
    structNAVINFO navinfo = myhandler.positionStamp;
    myhandler.navinfoMutex.unlock();

	unsigned int spd, spdd = 0, yr, rc;
	unsigned int ack = 2030, lr = 20, mr = 90, ridarheight = 50/*0 - 125cm*/, bw = 0x47;
	float speed = 0;//
	float yawRate = 0; //
	float radius = 8191;//todo


//    speed = navinfo.mSpeed3d;
//    yawRate = navinfo.mAngularRateZ;

	if (speed < 0) {    //send car speed and speed direction
		speed = -speed;
		spdd = 0;
	}

	if (speed > 127.9375) speed = 127.9375; //max speed value

	spd = (unsigned int)(speed / 0.0625); //speed * 16 (<< 4)

	if (yawRate >= 0){                  //heading rate
		if (yawRate > 127.9375) yawRate = 127.9375;//max heading rate
		yr = (unsigned int)(yawRate / 0.0625); //heading rate * 16(<< 4)
	}
	else{
		yawRate = -yawRate;
		if (yawRate > 128) yawRate = 128;
		yr = (((unsigned int)(yawRate / 0.0625) - 1) ^ 0x07FF) | 0x0800; //
	}
	if (radius >= 0){
		if (radius > 8191) radius = 8191;//curvature radius
		rc = (unsigned int)(radius);
	}
	else{
		radius = -radius;
		if (radius > 8192) radius = 8192;
		rc = (((unsigned int)(radius)-1) ^ 0x1FFF) | 0x2000;
	}

	can_frame frame[4];
	frame[0].can_id = 0x4F0;
	frame[0].can_dlc = 8;
	frame[0].data[0] = (spd >> 3);//speed high 8
	frame[0].data[1]=((spd & 0x07) << 5) | ((spdd & 0x01) << 4) | (yr >> 8);//speed low 3  + spdd low 1 + yr high 4
	frame[0].data[2]=yr & 0xFF;//yr low 8
	frame[0].data[3]=0x80 | (rc >> 8);
	frame[0].data[4]=rc & 0xFF;
	frame[0].data[5]=0;
	frame[0].data[6]=0;
	frame[0].data[7]=0;
	int nbytes0 = 0;
	nbytes0 = write(C1, &frame[0], sizeof(frame[0]));
	if(nbytes0 != sizeof(frame[0]))
	{
		std::cout << "CAN Send frame0 ERROR!" << std::endl;
	}

	frame[1].can_id = 0x4F1;
	frame[1].can_dlc = 8;
	frame[1].data[0]= ack >> 8;//SCAN_INDEX_ACK
	frame[1].data[1]= ack & 0xFF;
	frame[1].data[2]=0;
	frame[1].data[3]=0;
	frame[1].data[4]=0;
	frame[1].data[5]=0;
	frame[1].data[6]=0xBF;//RADAR_CMD_RADIATE  MAXIMUM_TRACKS
	frame[1].data[7]=0x20;//VEHICLE_SPEED_VALIDITY
	int nbytes1 = 0;
	nbytes1 = write(C1, &frame[1], sizeof(frame[1]));
	if(nbytes1 != sizeof(frame[1]))
	{
		std::cout << "CAN Send frame1 ERROR!" << std::endl;
	}

	frame[2].can_id = 0x5F2;
	frame[2].can_dlc = 8;
	frame[2].data[0]=0;
	frame[2].data[1]=0;
	frame[2].data[2]=lr >> 1;//RADAR_FOV_LR 20
	frame[2].data[3]=((lr & 0x01) << 7) | (mr & 0x7F);//RADAR_FOV_MR 90
	frame[2].data[4]=ridarheight & 0x7F; //RADAR_HEIGHT 50cm
	frame[2].data[5]=0x3C;//AALIGN_AVG_CTR_TOTAL 1750  AUTO_ALIGN_CONVERGED 1
	frame[2].data[6]=0;
	frame[2].data[7]=0;
	int nbytes2 = 0;
	nbytes2 = write(C1, &frame[2], sizeof(frame[2]));
	if(nbytes2 != sizeof(frame[2]))
	{
		std::cout << "CAN Send frame2 ERROR!" << std::endl;
	}
	
	frame[3].can_id = 0x5F4;
	frame[3].can_dlc = 8;
	frame[3].data[0]=0;
	frame[3].data[1]=bw & 0x7F;//BEAMWIDTH_VERT 4.4375deg
	frame[3].data[2]=0;
	frame[3].data[3]=0;
	frame[3].data[4]=0;
	frame[3].data[5]=0;
	frame[3].data[6]=0;
	frame[3].data[7]=0;
	int nbytes3;
	nbytes3 = write(C1, &frame[3], sizeof(frame[3]));
	if(nbytes3 != sizeof(frame[3]))
	{
		std::cout << "CAN Send frame3 ERROR!" << std::endl;
	}
}



void Esr::inPath_ID_ACC_FCW(can_frame *Message){//0x4E3

	ACC_FCW.path_id_ACC = (Message->data[1]);
	ACC_FCW.path_id_ACC_stat = (Message->data[7]);
	ACC_FCW.path_id_CMBB_move = (Message->data[2]);
	ACC_FCW.path_id_CMBB_stat = (Message->data[3]);
	ACC_FCW.path_id_FCW_move = (Message->data[4]);
	ACC_FCW.path_id_FCW_stat = (Message->data[5]);
	// cout << "In-path ACC target ID(moving or moveable): " << (int)(Message->data[1])
 //         << " In-path ACC target ID(stationary oroncoming): " << (int)(Message->data[7]) <<endl; // ACC
	// cout << "In-path FCW target ID(moving): " << (int)(Message->data[4])
 //         << " In-path stationary FCW target ID: " << (int)(Message->data[5]) <<endl; // FCW
 //    cout << "In-path moving CMBB target ID: " << (int)(Message->data[2])
 //         << " In-path stationary CMBB target ID: " << (int)(Message->data[3]) <<endl; // CMBB
}


void Esr::canInfoRead(){
	int nbytes;
	struct can_frame frame[100];
	int rec = 50;

	while(1){
		for (int i = 0; i < rec; i++) { //receive 50 frame
			nbytes = read(C1, &frame[i], sizeof(frame[i]));

			if(nbytes <= 0){
				cout<<"no read data!!!!!!!"<<endl;
				return;
			}
			int flag = (frame + i)->can_id;
			if(flag == 0x4E0){
				stLstTop_mutex.lock();
				cout<<"~~~~~~~~~~~~~~~~~~~~start work~~~~~~~~~~~~~~~~~~~~~~~~~"<<stLstTop<<endl;
				stLstTop = (stLstTop + 1) % MsgHeapSize; //new frame start
				stLstTop_mutex.unlock();
			}
			else if(flag == 0x4E3){
				inPath_ID_ACC_FCW(&frame[i]);  
			}
			else if(flag == 0x540){
				for (size_t i = 0; i < 7; i++)
				{
                //group id range 0~9
				// cout<<"group id: "<<((Message.DATA[0] & 0xF)* 7 + i) <<endl;
					Tracks[(frame->data[0] & 0xF) * 7 + i] = frame->data[i + 1];//data[1~7] belong to isDynamicOrStatic flag
					if((frame->data[0] & 0xF) == 9){
						break;
					}
				}
			}
			else if(flag >= 0x500 && flag <= 0x53F){ //64 frame
				for (int msgI = 0; msgI < 8; msgI++)
					TrackInfo[stLstTop][flag - 0x0500][msgI] = frame->data[msgI];//128 64 8
			}
			else{
			// printf("no Message ID  0x%x !!!!!!!!!\n",flag);
			}

		}
		usleep(10*1000);
	}
}


void start_read(Esr* esr){
	esr->canInfoRead();
}


void zcm_fun(){
    if (!myzcmSub.good())
        return;
    myzcmSub.subscribe("NAVINFO", &Handler::handleNAVINFOMessage, &myhandler);
    myzcmSub.run();
}


void start_esr(Esr * esr){
	for (size_t i = 0; i < NUM_OBJ; i++)
	{
		objWeights[i] = 0;
	}
	
	int stLstWork = 0;
	// Esr *esr = new Esr();
	while(true){
		esr->canInfoWrite();//write data to can
        cout<<"--------------------start waiting----------------------"<<stLstWork<<endl;
        stLstTop_mutex.lock();
		stLstWork = stLstTop;// stLstTop固定速率 + 1
        stLstTop_mutex.unlock();

		while (stLstWork == stLstTop){//waiting read a frame data
            usleep(10);
        };
        work(stLstWork); //process data
	}
}


int main(){

	Esr * esr = new Esr();
	thread navinfoRec(zcm_fun);//zcm receive position and timestamp
	thread recCanInfo(start_read,esr);//thread read objs data from can and save them
	thread startEsr(start_esr,esr);

	navinfoRec.join();
	startEsr.join();
	recCanInfo.join();

	return 0;

}

