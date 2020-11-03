/*
 * @Copyright (c) 2014-2019 TiEV :  All rights reserved.
 * @FileName: 
 * @Descripttion: 
 * @Author: Junqiao Zhao 
 * @version: 
 * @Date: 2019-10-27 
 * @FunctionList: 
 * @History: 
 * @LastEditors: Junqiao Zhao 
 * @LastEditTime: 2020-10-21
 */

#include "ESRControl.hpp"
#include "nature.h"

#define MapL TiEV::GRID_ROW
#define MapW TiEV::GRID_COL
#define OriginX TiEV::CAR_CEN_COL
#define OriginY TiEV::CAR_CEN_ROW
#define PerGrid TiEV::GRID_RESOLUTION
#define OFFSET_X 4
#define OFFSET_Y 15

ESRControl::ESRControl()
{
}

ESRControl::~ESRControl()
{
    //if(myEsrMap)
    //    delete myEsrMap;
    if(myEsrObjInfo)
        delete myEsrObjInfo;
    EHB_CAN_PORT = -1;
}

ESRControl::ESRControl(const int canPort)
{
    //myEsrMap = new structESRMAP;
    myEsrObjInfo = new structESROBJINFO;
    EHB_CAN_PORT = canPort;
}

void ESRControl::init()
{
    INFO("Start to read ESR info");
    static std::thread read_info(&ESRControl::canInfoRead, this);

	//INFO("Start to write Navinfo to ESR and generate ESRMAP");
	INFO("Start to write Navinfo to ESR and generate ESROBJINFO");
    static std::thread write_info(&ESRControl::startEsr, this);
}

void ESRControl::setNavInfo(nav_info_t& nav_info)
{
    std::lock_guard<std::mutex> lk(nav_info_lock);
    navInfo = nav_info;
}

void ESRControl::inPath_ID_ACC_FCW(can_frame *Message){//0x4E3

	ACC_FCW.path_id_ACC = (Message->data[1]);
	ACC_FCW.path_id_ACC_stat = (Message->data[7]);
	ACC_FCW.path_id_CMBB_move = (Message->data[2]);
	ACC_FCW.path_id_CMBB_stat = (Message->data[3]);
	ACC_FCW.path_id_FCW_move = (Message->data[4]);
	ACC_FCW.path_id_FCW_stat = (Message->data[5]);
}


//structESRMAP* ESRControl::getEsrMapPtr()
//{
//    return  myEsrMap;
//}

structESROBJINFO* ESRControl::getEsrObjInfoPtr()
{
    return  myEsrObjInfo;
}
void ESRControl::canInfoRead()
{
    int nbytes;
    struct can_frame frame[100];
    int rec = 50;
    int C1 = EHB_CAN_PORT;

    while (1)
    {
        for (int i = 0; i < rec; i++)
        { //receive 50 frame
            nbytes = read(C1, &frame[i], sizeof(frame[i]));

            if (nbytes <= 0)
            {
                cout << "no read data!!!!!!!" << endl;
                return;
            }
            int flag = (frame + i)->can_id;
            if (flag == 0x4E0)
            {//Message Heap, the other thread decode message from the top of the heap
                stLstTop_mutex.lock();
                cout << "~~~~~~~~~~~~~~~~~~~~start work~~~~~~~~~~~~~~~~~~~~~~~~~" << stLstTop << endl;
                stLstTop = (stLstTop + 1) % MsgHeapSize; //new frame start
                stLstTop_mutex.unlock();
            }
            else if (flag == 0x4E3)
            {//special object IDs for ACC and FCW (Forward Collision Warning system) and CMBB (Collsion Mitigation by Breaking) 
                inPath_ID_ACC_FCW(&frame[i]);
            }
            else if (flag == 0x540)
            {
                //group id: frame.data[0] & 0xF range 0~9
                //group 0~8 contain 7 tracks each (1~63) and group 9 contains track 64
                //cout<<"Track id: "<<((frame.data[0] & 0xF)* 7 + i) <<endl;
                if ((frame->data[0] & 0xF) == 9)
                {//the last group 9
                    Tracks[(frame->data[0] & 0xF) * 7] = frame->data[1]; //data[1~7] belong to isDynamicOrStatic flag
                }
                else 
                {//group 0 ~ 8
                    for (size_t i = 0; i < 7; i++)
                        Tracks[(frame->data[0] & 0xF) * 7 + i] = frame->data[i + 1]; //data[1~7] belong to isDynamicOrStatic flag
                }
            }
            else if (flag >= 0x500 && flag <= 0x53F)
            { //53F-500 = 64 Track information 
                for (int msgI = 0; msgI < 8; msgI++)
                    TrackInfo[stLstTop][flag - 0x0500][msgI] = frame->data[msgI]; //128 64 8
            }
            else
            {
                // printf("no Message ID  0x%x !!!!!!!!!\n",flag);
            }
        }
        usleep(10 * 1000);
    }
}

//decode a message from the message heap
int ESRControl::work(int messageNum){

	int ObjectID;
	int angle, range, width, rangespd, latspd, bridge;
	double angleR, rangeR, widthR, rangespdR, latspdR;
	double objX, objY, objB, objE;

    //Filtering for stability
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

        //the type fileter of the objects
//        if (ObjectID == ACC_FCW.path_id_CMBB_stat ||
//            ObjectID == ACC_FCW.path_id_CMBB_move ||
//            ObjectID == ACC_FCW.path_id_FCW_stat ||
//            ObjectID == ACC_FCW.path_id_FCW_move ||
//            ObjectID == ACC_FCW.path_id_ACC ||
//            ObjectID == ACC_FCW.path_id_ACC_stat)
            // cout<<"Tracks[ObjectID] & 0x20: "<<((Tracks[ObjectID] & 0x20)>>5)<<endl;
			//if ((((Tracks[ObjectID] & 0x20)>>5) != 0 && (int)(Tracks[ObjectID] & 0x80) != 0))
		// || objWeights[ObjectID] > 0){

        //Description:
        // Tracks[ObjectID] & 0x80)>>7 : Moving   
        // Tracks[ObjectID] & 0x40)>>6 : Movable Fast   
        // Tracks[ObjectID] & 0x20)>>5 : Movable Slow   
        // Tracks[ObjectID] & 0x10)>>4 : Tracking power -10~21 dB the higher the better   
        //if (((Tracks[ObjectID] & 0x20)>>5) != 0)
        cout<<"ObjectID: " << ObjectID << " Moving: " << (Tracks[ObjectID] & 0x80>>7) << " Movable Fast: " << (Tracks[ObjectID] & 0x40>>6)<< " Movable Slow: " << (Tracks[ObjectID] & 0x20>>5)<<endl;

        if (TrackInfo[messageNum][ObjectID][0] == 0 && TrackInfo[messageNum][ObjectID][1] == 0 && TrackInfo[messageNum][ObjectID][2] == 0 && TrackInfo[messageNum][ObjectID][3] == 0)
            continue; //no data

        objWeights[ObjectID] = FILTER;
        //decode 0x500-0x53F not a full list, others include status (tracking status), oncoming, group change etc.
        latspd = ((TrackInfo[messageNum][ObjectID][0] & 0xFC) >> 2);                                                    //横向速度
        angle = ((TrackInfo[messageNum][ObjectID][1] & 0x1F) << 5) + ((TrackInfo[messageNum][ObjectID][2] & 0xF8) >> 3); //偏航角
        range = ((TrackInfo[messageNum][ObjectID][2] & 0x07) << 8) + TrackInfo[messageNum][ObjectID][3];                  //距离
        width = ((TrackInfo[messageNum][ObjectID][4] & 0x3C) >> 2);                                                       //宽度
        bridge = ((TrackInfo[messageNum][ObjectID][4] & 0x80) >> 7);                                                      //bridge
        rangespd = ((TrackInfo[messageNum][ObjectID][6] & 0x3F) << 8) + TrackInfo[messageNum][ObjectID][7];               //纵向速度

        //according to the document verify
        if (angle < (1 << 9))     //51.2 * 10 degree
            angleR = angle * 0.1; //angle > 0
        else
            angleR = (angle - (1 << 10)) * 0.1; //angle < 0

        angleR = angleR * 3.14159 / 180.0; //convert to rad
        rangeR = range * 0.1;              //get the value of obj's range
        widthR = width * 0.5;              //get the value of obj's width

        if (rangespd < (1 << 13))
            rangespdR = rangespd * 0.01; //v > 0
        else
            rangespdR = (rangespd - (1 << 14)) * 0.01; //v < 0
        //
        if (latspd < (1 << 5))
            latspdR = latspd * 0.25;
        else
            latspdR = (latspd - (1 << 6)) * 0.25;

        double rangespd_h = rangespdR * sin(angleR) + latspdR * cos(angleR); //get the horizontal speed
        double rangespd_v = rangespdR * cos(angleR) + latspdR * sin(angleR); //get the vertical speed
        objX = OriginX + rangeR * sin(angleR) / PerGrid;                     //get the X position of obj in gridmap
        objY = OriginY - rangeR * cos(angleR) / PerGrid;                     //get the Y position of obj in gridmap

        objArr[ObjectID].objWidth = widthR;
        if (((Tracks[ObjectID] & 0x80)>>7) != 0)
        {//Moving 
            //Filtering dynamic object for stability
            if (sqrt((objArr[ObjectID].objH - objX) * (objArr[ObjectID].objH - objX) +
                     (objArr[ObjectID].objV - objY) * (objArr[ObjectID].objV - objY))
                > FILTER_ASSOCIATION ) //this obj distance moved from last frame to current frame > 25
            {
            	if (abs(objArr[ObjectID].speedV) > TOL || abs(objArr[ObjectID].speedH) > TOL)
            	{
            		oldObj.push_back(objArr[ObjectID]);
            	}
                //OFFSET_X 4 and OFFSET_Y 15 are calibrated number for aligning to the vehicle frame
                objArr[ObjectID].objH = objX - OFFSET_X;//update the position of current obj in gridmap
		objArr[ObjectID].objV = objY - OFFSET_Y;
            } else //distance moved < 25
            {
                //4 and 15 are calibrated number for aligning to the vehicle frame
                objArr[ObjectID].objH = 0.5 * (objX - OFFSET_X) + 0.5 * objArr[ObjectID].objH;//middle value
                objArr[ObjectID].objV = 0.5 * (objY - OFFSET_Y) + 0.5 * objArr[ObjectID].objV;
            }
            objArr[ObjectID].speedH = rangespd_h;
            objArr[ObjectID].speedV = rangespd_v;

            //size of the object
            objX = objArr[ObjectID].objH;
            objY = objArr[ObjectID].objV;
            if (objY < 0) continue;  //y top not in map
            objB = objX - (widthR * 0.5 / PerGrid);
            if (objB < 0) continue;  //x left not in map
            objE = objX + (widthR * 0.5 / PerGrid);
            if (objE > MapW) continue;//x right not in map


            //for (int j = 0; j <= abs((speed-rangespdR)/ SpdN) + 1; j++)
 
            // //generate esr map
            // esr_map_lock.lock();
            // for (int j = 0; j <= 11; j++)//height suppose to be 12
            // {
            //     for (int i = (int) objB; i <= (int) objE + 10; i++)//width suppose to be (widthR + 10)
            //     {
            //     	myEsrMap->cells[(int) objY + j][i] = 255;
            //         // mapEsr.ptr<uchar>((int) objY + j)[i] = 255/*((int(widthR / PerGrid) == 0) ? 100 : 240)*/;
            //     }
            // }
            // esr_map_lock.unlock();
        }
        else
        {//not moving
            // filtering for judging dynamic object > 0 means dynamic 
        	objWeights[ObjectID]--;
            //what are these? John 2020.10.23
        	objArr[ObjectID].objV -= (objArr[ObjectID].speedV * 0.05) / PerGrid;
			objArr[ObjectID].objH -= (objArr[ObjectID].speedH * 0.05) / PerGrid;
        	//cout << "objV: " << objArr[ObjectID].objV << " objH: " << objArr[ObjectID].objH << endl;
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

                // esr_map_lock.lock();
				// //for (int j = 0; j <= abs((speed-rangespdR)/ SpdN) + 1; j++)
        		// for (int j = 0; j <= 5; j++)
        		// 	for (int i = (int)objB; i <= (int)objE + 4; i++)
        		// 		myEsrMap->cells[(int) objY + j][i] = ((int(widthR / PerGrid) == 0) ? 100 : 240);
        		// 		// mapEsr.ptr<uchar>((int)objY + j)[i] = ((int(widthR / PerGrid) == 0) ? 100 : 240);
                // esr_map_lock.unlock();
        		
        	}
        }

        cout << "objV: " << objArr[ObjectID].objV << " objH: " << objArr[ObjectID].objH << endl;
        cout << "speedV: " << objArr[ObjectID].speedV << ' ' << "speedH: " << objArr[ObjectID].speedH << endl;
        cout << "width: " << widthR << endl;
    }
    
    //draw car
    // for(int i = 68; i<= 82;i++){
    // 	myEsrMap->cells[300][i] = 255;
    // 	myEsrMap->cells[320][i] = 255;
    // 	// mapEsr.ptr<uchar>(300)[i] = 255;
    // 	// mapEsr.ptr<uchar>(320)[i] = 255;
    // }
    // for(int j = 300;j<=320;j++){
    // 	myEsrMap->cells[j][68] = 255;
    // 	myEsrMap->cells[j][82] = 255;
    // 	// mapEsr.ptr<uchar>(j)[68] = 255;
    // 	// mapEsr.ptr<uchar>(j)[82] = 255;
    // }
    // 

    //outputMap(); //all the 64 obj can view in the map
    // esr_map_lock.lock();
    // int objNum = 0; //every time start new counts form 0, so send the current newest objs???
    // myEsrMap->timestamp = TiEV::getTimeStamp();
    // myEsrMap->mHeading = navInfo.mHeading;//position and timestamp info
    // myEsrMap->utmX = navInfo.utmX;
    // myEsrMap->utmY = navInfo.utmY;
    // myEsrMap->resolution = TiEV::GRID_RESOLUTION;
    // myEsrMap->rows = TiEV::GRID_ROW;
    // myEsrMap->cols = TiEV::GRID_COL;
    // myEsrMap->center_col = TiEV::CAR_CEN_COL;
    // myEsrMap->center_row = TiEV::CAR_CEN_ROW;
    // esr_map_lock.unlock();

    int count = 0;//esrObjInfo.Count;//current obj counts

    esr_objinfo_lock.lock();
    myEsrObjInfo->timestamp = TiEV::getTimeStamp();
    for (size_t i = 0; i < NUM_OBJ; i++) //current 64 obj info save to esrObjInfo and objArrSel
    {
        if (objWeights[i] > 0)//is dynamic obj
        {
            myEsrObjInfo->m_number_objs = ++count;
            myEsrObjInfo->objects[count].m_horizon = objArr[i].objH;
            myEsrObjInfo->objects[count].m_vertical = objArr[i].objV;
            myEsrObjInfo->objects[count].m_horizon_velocity = objArr[i].speedH;
            myEsrObjInfo->objects[count].m_vertical_velocity = objArr[i].speedV;
            myEsrObjInfo->objects[count].m_width = objArr[i].objWidth;
            myEsrObjInfo->objects[count].m_objtype = 0; 
            if (i ==  ACC_FCW.path_id_ACC_stat)
                myEsrObjInfo->objects[count].m_objtype = 1; 
            else if(i == ACC_FCW.path_id_ACC)
                myEsrObjInfo->objects[count].m_objtype = 1; 
            else if(i ==  ACC_FCW.path_id_CMBB_move)
                myEsrObjInfo->objects[count].m_objtype = 3; 
            else if(i == ACC_FCW.path_id_CMBB_stat)
                myEsrObjInfo->objects[count].m_objtype = 3; 
            else if(i == ACC_FCW.path_id_FCW_move)
                myEsrObjInfo->objects[count].m_objtype = 2; 
            else if(i == ACC_FCW.path_id_FCW_stat)
                myEsrObjInfo->objects[count].m_objtype = 2; 
            // if (++objNum == 64) break;
        }
    }
    esr_objinfo_lock.unlock();

    usleep(10);
    return 0;
}

void ESRControl::startEsr(){
	for (size_t i = 0; i < NUM_OBJ; i++)
	{
		objWeights[i] = 0;
	}
	
	int stLstWork = 0;
	while(true){
		canInfoWrite();//write data to can
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

void ESRControl::canInfoWrite()
{ 
        unsigned int spd, spdd = 0, yr, rc;
        unsigned int ack = 2030, lr = 20, mr = 90, ridarheight = 50 /*0 - 125cm*/, bw = 0x47;
        float speed = 0;     //
        float yawRate = 0;   //
        float radius = 8191; //todo
        int C1 = EHB_CAN_PORT;

        nav_info_lock.lock();
        speed = navInfo.speed;
        yawRate = navInfo.yawRate;
        nav_info_lock.unlock();

        if (speed < 0)
        { //send car speed and speed direction
            speed = -speed;
            spdd = 0;
        }

        if (speed > 127.9375)
            speed = 127.9375; //max speed value

        spd = (unsigned int)(speed / 0.0625); //speed * 16 (<< 4)

        if (yawRate >= 0)
        { //heading rate
            if (yawRate > 127.9375)
                yawRate = 127.9375;                //max heading rate
            yr = (unsigned int)(yawRate / 0.0625); //heading rate * 16(<< 4)
        }
        else
        {
            yawRate = -yawRate;
            if (yawRate > 128)
                yawRate = 128;
            yr = (((unsigned int)(yawRate / 0.0625) - 1) ^ 0x07FF) | 0x0800; //
        }
        if (radius >= 0)
        {
            if (radius > 8191)
                radius = 8191; //curvature radius
            rc = (unsigned int)(radius);
        }
        else
        {
            radius = -radius;
            if (radius > 8192)
                radius = 8192;
            rc = (((unsigned int)(radius)-1) ^ 0x1FFF) | 0x2000;
        }

        can_frame frame[4];
        frame[0].can_id = 0x4F0;
        frame[0].can_dlc = 8;
        frame[0].data[0] = (spd >> 3);                                             //speed high 8
        frame[0].data[1] = ((spd & 0x07) << 5) | ((spdd & 0x01) << 4) | (yr >> 8); //speed low 3  + spdd low 1 + yr high 4
        frame[0].data[2] = yr & 0xFF;                                              //yr low 8
        frame[0].data[3] = 0x80 | (rc >> 8);
        frame[0].data[4] = rc & 0xFF;
        frame[0].data[5] = 0;
        frame[0].data[6] = 0;
        frame[0].data[7] = 0;
        int nbytes0 = 0;
        nbytes0 = write(C1, &frame[0], sizeof(frame[0]));
        if (nbytes0 != sizeof(frame[0]))
        {
            std::cout << "CAN Send frame0 ERROR!" << std::endl;
        }

        frame[1].can_id = 0x4F1;
        frame[1].can_dlc = 8;
        frame[1].data[0] = ack >> 8; //SCAN_INDEX_ACK
        frame[1].data[1] = ack & 0xFF;
        frame[1].data[2] = 0;
        frame[1].data[3] = 0;
        frame[1].data[4] = 0;
        frame[1].data[5] = 0;
        frame[1].data[6] = 0xBF; //RADAR_CMD_RADIATE  MAXIMUM_TRACKS
        frame[1].data[7] = 0x20; //VEHICLE_SPEED_VALIDITY
        int nbytes1 = 0;
        nbytes1 = write(C1, &frame[1], sizeof(frame[1]));
        if (nbytes1 != sizeof(frame[1]))
        {
            std::cout << "CAN Send frame1 ERROR!" << std::endl;
        }

        frame[2].can_id = 0x5F2;
        frame[2].can_dlc = 8;
        frame[2].data[0] = 0;
        frame[2].data[1] = 0;
        frame[2].data[2] = lr >> 1;                          //RADAR_FOV_LR 20
        frame[2].data[3] = ((lr & 0x01) << 7) | (mr & 0x7F); //RADAR_FOV_MR 90
        frame[2].data[4] = ridarheight & 0x7F;               //RADAR_HEIGHT 50cm
        frame[2].data[5] = 0x3C;                             //AALIGN_AVG_CTR_TOTAL 1750  AUTO_ALIGN_CONVERGED 1
        frame[2].data[6] = 0;
        frame[2].data[7] = 0;
        int nbytes2 = 0;
        nbytes2 = write(C1, &frame[2], sizeof(frame[2]));
        if (nbytes2 != sizeof(frame[2]))
        {
            std::cout << "CAN Send frame2 ERROR!" << std::endl;
        }

        frame[3].can_id = 0x5F4;
        frame[3].can_dlc = 8;
        frame[3].data[0] = 0;
        frame[3].data[1] = bw & 0x7F; //BEAMWIDTH_VERT 4.4375deg
        frame[3].data[2] = 0;
        frame[3].data[3] = 0;
        frame[3].data[4] = 0;
        frame[3].data[5] = 0;
        frame[3].data[6] = 0;
        frame[3].data[7] = 0;
        int nbytes3;
        nbytes3 = write(C1, &frame[3], sizeof(frame[3]));
        if (nbytes3 != sizeof(frame[3]))
        {
            std::cout << "CAN Send frame3 ERROR!" << std::endl;
        }
}
