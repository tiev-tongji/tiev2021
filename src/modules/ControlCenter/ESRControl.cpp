#include "ESRControl.hpp"
#include "nature.h"


ESRControl::ESRControl()
{
}

ESRControl::~ESRControl()
{
    if(myEsrMap)
        delete myEsrMap;
    EHB_CAN_PORT = -1;
}

ESRControl::ESRControl(const int canPort)
{
    myEsrMap = new structESRMAP;
    EHB_CAN_PORT = canPort;
}

void ESRControl::init()
{
    INFO("Start to read ESR info");
    static std::thread read_info(&ESRControl::canInfoRead, this);

	INFO("Start to write Navinfo to ESR and generate ESRMAP");
    static std::thread write_info(&ESRControl::startEsr, this);
}

void ESRControl::setNavInfo(nav_info_t& nav_info)
{
    std::lock_guard<std::mutex> lk(nav_info_lock);
    navInfo = nav_info;
}

structESRMAP* ESRControl::getEsrMapPtr()
{
    return  myEsrMap;
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
            {
                stLstTop_mutex.lock();
                cout << "~~~~~~~~~~~~~~~~~~~~start work~~~~~~~~~~~~~~~~~~~~~~~~~" << stLstTop << endl;
                stLstTop = (stLstTop + 1) % MsgHeapSize; //new frame start
                stLstTop_mutex.unlock();
            }
            else if (flag == 0x4E3)
            {
                inPath_ID_ACC_FCW(&frame[i]);
            }
            else if (flag == 0x540)
            {
                for (size_t i = 0; i < 7; i++)
                {
                    //group id range 0~9
                    // cout<<"group id: "<<((Message.DATA[0] & 0xF)* 7 + i) <<endl;
                    mvLst[(frame->data[0] & 0xF) * 7 + i] = frame->data[i + 1]; //data[1~7] belong to isDynamicOrStatic flag
                    if ((frame->data[0] & 0xF) == 9)
                    {
                        break;
                    }
                }
            }
            else if (flag >= 0x500 && flag <= 0x53F)
            { //64 frame
                for (int msgI = 0; msgI < 8; msgI++)
                    stLst[stLstTop][flag - 0x0500][msgI] = frame->data[msgI]; //128 64 8
            }
            else
            {
                // printf("no Message ID  0x%x !!!!!!!!!\n",flag);
            }
        }
        usleep(10 * 1000);
    }
}

int ESRControl::work(int num){
	int workID;
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
	for (workID = 0; workID < NUM_OBJ; workID++){

//        if (workID == ACC_FCW.path_id_CMBB_stat ||
//            workID == ACC_FCW.path_id_CMBB_move ||
//            workID == ACC_FCW.path_id_FCW_stat ||
//            workID == ACC_FCW.path_id_FCW_move ||
//            workID == ACC_FCW.path_id_ACC ||
//            workID == ACC_FCW.path_id_ACC_stat)
            // cout<<"mvLst[workID] & 0x20: "<<((mvLst[workID] & 0x20)>>5)<<endl;
			//if ((((mvLst[workID] & 0x20)>>5) != 0 && (int)(mvLst[workID] & 0x80) != 0))
		// || objWeights[workID] > 0){
        if (((mvLst[workID] & 0x20)>>5) != 0)//is dynamic obj
        {
            if (stLst[num][workID][0] == 0
                && stLst[num][workID][1] == 0
                && stLst[num][workID][2] == 0
                && stLst[num][workID][3] == 0)
                continue; //no data


            cout << "------------------------------------" << endl;
            //if (((int)(mvLst[workID] & 0x20) != 0 && (int)(mvLst[workID] & 0x80) != 0)) {
            cout << workID << " SEE A DYNAMIC OBJ!!!!!!!" << endl;
            objWeights[workID] = FILTER;

            latspd = ((stLst[num][workID][0] & 0xFC) >> 2); //横向速度?
            angle = ((stLst[num][workID][1] & 0x1F) << 5) + ((stLst[num][workID][2] & 0xF8) >> 3);
            range = ((stLst[num][workID][2] & 0x07) << 8) + stLst[num][workID][3];//
            width = ((stLst[num][workID][4] & 0x3C) >> 2);
            bridge = ((stLst[num][workID][4] & 0x80) >> 7);
            rangespd = ((stLst[num][workID][6] & 0x3F) << 8) + stLst[num][workID][7];

            //	cout << "fast moving ID:  " << workID << endl;
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

            if (sqrt((objArr[workID].objH - objX) * (objArr[workID].objH - objX) +
                     (objArr[workID].objV - objY) * (objArr[workID].objV - objY))
                > 25) //this obj distance moved from last frame to current frame > 25
            {
            		//	cout << "left id:" << workID << " speedH:" << objArr[workID].speedH << " speedV:" << objArr[workID].speedV << endl;
            	if (abs(objArr[workID].speedV) > TOL || abs(objArr[workID].speedH) > TOL)
            	{
            		oldObj.push_back(objArr[workID]);
            	}
                objArr[workID].objH = objX - 4;//update the position of current obj in gridmap but why x - 4 and y - 15
                objArr[workID].objV = objY - 15;
            } else //distance moved < 25
            {
                objArr[workID].objH = 0.5 * (objX - 4) + 0.5 * objArr[workID].objH;//middle value?
                objArr[workID].objV = 0.5 * (objY - 15) + 0.5 * objArr[workID].objV;
            }
            ++numobj;

            //todo
            objX = objArr[workID].objH;
            objY = objArr[workID].objV;
            if (objY < 0) continue;  //y top not in map
            objB = objX - (widthR * 0.5 / PerGrid);
            if (objB < 0) continue;  //x left not in map
            objE = objX + (widthR * 0.5 / PerGrid);
            if (objE > MapW) continue;//x right not in map



            // cout << workID << " SEE A DYNAMIC OBJ!!!!!!!" << endl;
            cout << "objV: " << objArr[workID].objV << " objH: " << objArr[workID].objH << endl;
            objArr[workID].speedH = rangespd_h;
            objArr[workID].speedV = rangespd_v;
            cout << "speedV: " << objArr[workID].speedV << ' ' << "speedH: " << objArr[workID].speedH << endl;
            cout << "width: " << widthR * 0.5 << endl;
            cout << "ismove: " << ((mvLst[workID] & 0x20) >> 5) << endl;// 1
            cout << "current numobj: " << numobj << endl;
            //for (int j = 0; j <= abs((speed-rangespdR)/ SpdN) + 1; j++)
            esr_map_lock.lock();
            for (int j = 0; j <= 11; j++)//height suppose to be 12
            {
                for (int i = (int) objB; i <= (int) objE + 10; i++)//width suppose to be (widthR + 10)
                {
                	myEsrMap->cells[(int) objY + j][i] = 255;
                    // mapEsr.ptr<uchar>((int) objY + j)[i] = 255/*((int(widthR / PerGrid) == 0) ? 100 : 240)*/;
                }
            }
            esr_map_lock.unlock();
        }

        else
        {
        	cout << workID << " NOT SEE  Weight: " << objWeights[workID] << endl;
        	cout << "speedV: " << objArr[workID].speedV << ' ' << "speedH: " << objArr[workID].speedH << endl;
        	objWeights[workID]--;
        	objArr[workID].objV -= (objArr[workID].speedV * 0.05) / PerGrid;
			//objArr[workID].objH -= (objArr[workID].speedH * 0.05) / PerGrid;
        	cout << "objV: " << objArr[workID].objV << " objH: " << objArr[workID].objH << endl;
        	if (abs(objArr[workID].speedV) < TOL && abs(objArr[workID].speedH) < TOL)
        	{
        		objWeights[workID] = 0;
        		cout << "speed 0" << endl;
        		continue;
        	}
        	if (objArr[workID].objV < 0 || objArr[workID].objV > MapL || objArr[workID].objH < 0 || objArr[workID].objH > MapW)
        	{
        		objWeights[workID] = 0;
        		cout << "out of range" << endl;
        		continue;
        	}
        	else
        	{
        		objX = objArr[workID].objH;
        		objY = objArr[workID].objV;
        		if (objY < 0) continue;
        		objB = objX - (widthR*0.5 / PerGrid);
        		if (objB < 0) continue;
        		objE = objX + (widthR*0.5 / PerGrid);
        		if (objE > MapW) continue;

                esr_map_lock.lock();
				//for (int j = 0; j <= abs((speed-rangespdR)/ SpdN) + 1; j++)
        		for (int j = 0; j <= 5; j++)
        			for (int i = (int)objB; i <= (int)objE + 4; i++)
        				myEsrMap->cells[(int) objY + j][i] = ((int(widthR / PerGrid) == 0) ? 100 : 240);
        				// mapEsr.ptr<uchar>((int)objY + j)[i] = ((int(widthR / PerGrid) == 0) ? 100 : 240);
                esr_map_lock.unlock();
        		
        	}
        }
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
    esr_map_lock.lock();
    int objNum = 0; //every time start new counts form 0, so send the current newest objs???
    myEsrMap->timestamp = TiEV::getTimeStamp();
    myEsrMap->mHeading = navInfo.mHeading;//position and timestamp info
    myEsrMap->utmX = navInfo.utmX;
    myEsrMap->utmY = navInfo.utmY;
    myEsrMap->resolution = TiEV::GRID_RESOLUTION;
    myEsrMap->rows = TiEV::GRID_ROW;
    myEsrMap->cols = TiEV::GRID_COL;
    myEsrMap->center_col = TiEV::CAR_CEN_COL;
    myEsrMap->center_row = TiEV::CAR_CEN_ROW;
    esr_map_lock.unlock();

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
void ESRControl::inPath_ID_ACC_FCW(can_frame *Message) //0x4E3
{
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

 //         << " In-path stationary CMBB target ID: " << (int)(Message->data[3]) <<endl; // CMB

     
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
