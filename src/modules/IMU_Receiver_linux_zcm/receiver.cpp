//////////////////////////////////////////////////////////////////////////
//UDP broadcast receiver for Oxts IMU
//Author: John Zhao Xudong He
//Date: 2016.5.8
//////////////////////////////////////////////////////////////////////////
#include "receiver.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <math.h>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <mutex>
#include "common/coordinate_converter/coordinate_converter.h"
#include "common/nature.h"
#include <zcm/zcm-cpp.hpp>
#include "structNAVINFO.hpp"
#include "structCANINFO.hpp"

#define IP_IMU "195.0.0.84" //No use in UDP
#define PORT_IMU 3000
#define SIZE_BUF 72
#define WAITOUT 500 //10 seconds

using namespace std;
using namespace TiEV;
time_t now;
struct tm *timenow;
char Buffer_pos_path[256];
mutex mtx;
structNAVINFO navinfo; //the navigation struct
UTMCoor startpoint_utm{0, 0};
UTMCoor reckoncheck1{0, 0}, reckoncheck2{0, 0};
int rtkstatusnumber = 0; //��RTK��״̬�����˲�
double dCANspeed = 0.0;	 // received  can speed m/s
//for test reckon
std::ofstream reckoncheckfile1("reckonchekck1.txt"), rtkutm("rtkutm.txt");
//std::ofstream speedcheck("speedcheckfile.txt");
double heading_temp = 0;

class Handler
{
public:
	~Handler() {}

	void handleMessage(const zcm::ReceiveBuffer *rbuf,
					   const std::string &chan,
					   const structCANINFO *msg)
	{
		dCANspeed = (msg->carspeed / (360.0));
	}
};

/*
 threadfunc: listen pkts from IMU
 */
void listenfunc(NComRxC *nrx, bool bReport)
{
	if (nrx == NULL)
	{
		printf("NComRxC error!\n");
		return;
	}
	try
	{
		// 		WSASession Session;
		UDPSocket Socket;
		Socket.Bind(PORT_IMU, IP_IMU, false); //do not bind ip
		char buff[SIZE_BUF];
		FILE *fp_raw;
		if (bReport)
		{
			char ncom_path[256];
			strcpy(ncom_path, Buffer_pos_path);
			strcat(ncom_path, "_ncom.txt");
			//output the raw data
			fp_raw = fopen(ncom_path, "w");
			if (fp_raw == NULL)
			{
				printf("Cannot open imu_buffer.txt for writing!\n");
				return;
			}
		}

		while (1)
		{
			memset(&buff, 0, SIZE_BUF);
			Socket.RecvFrom((char *)&buff, SIZE_BUF);
			if (bReport)
			{
				//write out the raw data
				fwrite(buff, sizeof(char), SIZE_BUF, fp_raw);
			}
			mtx.lock();
			//Decode the package
			NComNewChars(nrx, (unsigned char *)&buff, SIZE_BUF);
			mtx.unlock();
		}
		fclose(fp_raw);
	}
	catch (std::exception &ex)
	{
		std::cout << ex.what();
	}
}

//modified by hexudong
void NEDIncre2BLHIncre(const double currentLati /*in*/, const double currentLongti /*in*/, const double northIncre /*in*/, const double eastIncre /*in*/,
					   double &reckoningLati /*out*/, double &reckoningLongti /*out*/)
{

	TiEV::Angle latAngle, lonAngle;
	latAngle.setByDegree(currentLati);
	lonAngle.setByDegree(currentLongti);
	TiEV::WGS84Coor latLonP((TiEV::LAT)latAngle, (TiEV::LON)lonAngle);
	TiEV::UTMCoor coord;
	coord = TiEV::latLonToUTMXY(latLonP);
	coord.x = coord.x + eastIncre;
	coord.y = coord.y + northIncre;
	//
	TiEV::WGS84Coor latLonOut;
	latLonOut = TiEV::UTMXYToLatLon(TiEV::UTMCoor(coord.x, coord.y));

	reckoningLati = latLonOut.lat.getDegree();
	reckoningLongti = latLonOut.lon.getDegree();
}

bool is_RTK_valid(NComRxC *nrx)
{ //Differential RTK float RTK Integer
	return (nrx->mIsGpsPosModeValid && (nrx->mGpsPosMode == 5 || nrx->mGpsPosMode == 6 || nrx->mGpsPosMode == 4) && nrx->mIsLatValid && nrx->mGpsNumObs >= 5);
}

/*
 reportfunc: report navinfo to virtualswitchbus
 */
void reportfunc(NComRxC *nrx, bool bReport, zcm::ZCM &zcm_publish)
{
	if (nrx == NULL)
	{
		printf("NComRxC error!\n");
		return;
	}
	//output the pos data
	FILE *fp_pos;
	if (bReport)
	{
		char pos_path[256];
		strcpy(pos_path, Buffer_pos_path);
		strcat(pos_path, "_pos.txt");
		fp_pos = fopen(pos_path, "w");
		if (fp_pos == NULL)
		{
			printf("Cannot open imu_pos.txt for writing!\n");
			return;
		}
	}
	//GPS time
	int i = 0;
	double gps2machine, mMachineTime;
	time_t t1;
	struct tm *td;
	int ms;

	// Convert GPS seconds (from 1980-01-06 00:00:00) to machine seconds (from 1970-01-01 00:00:00). It is
	// very likely the machine will adjust for leap seconds, hence the correct GPS UTC difference is
	// applied. If the local machine time does not start from 1970-01-01 00:00:00 then the value of
	// gps2machine below needs to change.
	gps2machine = 315964800.0;
	//
	bool bSetLocalTime = false;
	//
	//reckon distance starting from where rtk lost
	double reckoningDist = 0;
	//10 points to tell vehicle initial position when starting compute reckonx\reckony
	int startGuidepoints = 10;
	//reckon time, corrected CAN speed
	double dt = 0, Corrected_CANspeed = 0;
	//reckon  variables
	double north_meterIncre = 0, east_meterIncre = 0, reckoningLati = 0, reckoningLongti = 0;
	double north_meterIncre_temp = 0, east_meterIncre_temp = 0;
	//
	string infoDetail;
	struct timeval tpstart, tpend;
	float elapsedtime;
	gettimeofday(&tpstart, NULL);

	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
	int start_pts = 20;

	while (1)
	{
		mtx.lock();
		//when the GPS time is avaliable synchronise the system time with the GPS time
		if (nrx->mIsTimeValid)
		{
			mMachineTime = nrx->mTime + gps2machine + nrx->mTimeUtcOffset;

			//sec to microsec
			mMachineTime *= 1e6;
			navinfo.timestamp = mMachineTime;
		}
		// cout<<"nrx->mIsLatApprox: " << nrx->mIsLatApprox<< "  nrx->mIsLatValid :"<<nrx->mIsLatValid << " nrx->mIsLonValid "<<nrx->mIsLonValid<<endl;
		// cout<<"nrx->mIsGnssGpsEnabledValid: " << nrx->mIsGnssGpsEnabledValid <<endl;
		// cout<<"nrx->mPDOP: " << nrx->mPDOP <<endl;
		// cout<<"nrx->mHDOP: " << nrx->mHDOP <<endl;

		bool rtkValid = is_RTK_valid(nrx);

		//if (nrx->mIsLatValid && nrx->mGpsNumObs >= 5) //if contain valid data
		if (rtkValid && rtkstatusnumber == 0)
		{
			// cout<<"if (nrx->mIsLatValid) "<<endl;
			navinfo.mLat = nrx->mLat;
			navinfo.mLon = nrx->mLon;
			navinfo.mAlt = nrx->mAlt;
			//convert to UTM coordinates modifed by John 2017.10
			TiEV::Angle latAngle, lonAngle;
			latAngle.setByDegree(navinfo.mLat);
			lonAngle.setByDegree(navinfo.mLon);
			TiEV::WGS84Coor latLonP((TiEV::LAT)latAngle, (TiEV::LON)lonAngle);
			TiEV::UTMCoor coord;
			coord = TiEV::latLonToUTMXY(latLonP);
			navinfo.utmX = coord.x;
			navinfo.utmY = coord.y;
		}
		cout << " ----------------nrx->mIsHeadingValid: " << nrx->mIsHeadingValid << endl;
		if (nrx->mIsHeadingValid)
		{
			nrx->mHeading += 1.5;
			if (nrx->mHeading >= 360)
				nrx->mHeading -= 360;
			//modified by John 2017.10
			navinfo.mHeading = TiEV::ned_deg2iso_rad(nrx->mHeading);
			heading_temp = nrx->mHeading;
		}
		if (nrx->mIsPitchValid)
		{ //added by John 20161009 to assist false alert elimination
			navinfo.mPitch = nrx->mPitch;
		}
		if (nrx->mIsSpeed3dValid)
		{
			navinfo.mSpeed3d = nrx->mSpeed3d;
		}
		if (nrx->mIsVeValid)
		{
			navinfo.mVe = nrx->mVe;
		}
		if (nrx->mIsVnValid)
		{
			navinfo.mVn = nrx->mVn;
		}
		if (nrx->mIsWzValid)
		{
			//modified by John 2017.10
			navinfo.mAngularRateZ = -TiEV::deg2rad(nrx->mWz);
		}
		if (nrx->mIsCurvatureValid)
		{
			navinfo.mCurvature = nrx->mCurvature;
		}
		else
		{
			if (navinfo.mSpeed3d > 0)
			{
				navinfo.mCurvature = navinfo.mAngularRateZ / navinfo.mSpeed3d;
			}
		}
		if (nrx->mIsNorthAccValid && nrx->mIsEastAccValid)
		{
			navinfo.mHPOSAccuracy = sqrt(nrx->mNorthAcc * nrx->mNorthAcc + nrx->mEastAcc * nrx->mEastAcc);
		}
		//get the number of satellites.
		if (nrx->mIsGpsNumObsValid)
		{
			navinfo.mGpsNumObs = nrx->mGpsNumObs;
		}
		//get the x and y axis acceleration(modified by chenkai)
		if (nrx->mIsAxValid)
		{
			navinfo.mAx = nrx->mAx;
		}
		if (nrx->mIsAyValid)
		{
			navinfo.mAy = nrx->mAy;
		}

		//0.01 is the scale factor TOBE tested
		Corrected_CANspeed = dCANspeed * 1.01;
		cout << "Corrected_CANspeed:  " << Corrected_CANspeed << endl;
		//compute elapsedtime
		gettimeofday(&tpend, NULL);
		elapsedtime = (1000000 * (tpend.tv_sec - tpstart.tv_sec) + tpend.tv_usec - tpstart.tv_usec) / 1000000.0;
		gettimeofday(&tpstart, NULL);

		dt = elapsedtime;
		//always compute reckon increment
		north_meterIncre_temp = cos(heading_temp * M_PI / 180) * Corrected_CANspeed * dt;
		east_meterIncre_temp = sin(heading_temp * M_PI / 180) * Corrected_CANspeed * dt;
		north_meterIncre = navinfo.mVn / sqrt(navinfo.mVe * navinfo.mVe + navinfo.mVn * navinfo.mVn) * Corrected_CANspeed * dt;
		east_meterIncre = navinfo.mVe / sqrt(navinfo.mVe * navinfo.mVe + navinfo.mVn * navinfo.mVn) * Corrected_CANspeed * dt;
		// acceleration
		cout << "---------------------------acceleration------------------------------------------------" << endl;
		cout << "x axis acceleration:" << navinfo.mAx << "y axis acceleration:" << navinfo.mAy << endl;
		// heading search status
		cout << "---------------------------heading search status---------------------------------------" << endl;
		cout << "mHeadQuality:" << (int)(nrx->mHeadQuality) << " mHeadSearchType:" << (int)(nrx->mHeadSearchType) << " mHeadSearchStatus:" << (int)(nrx->mHeadSearchStatus) << " mHeadSearchReady:" << (int)(nrx->mHeadSearchReady) << endl;
		cout << "mHeadSearchInit:" << nrx->mHeadSearchInit << " mHeadSearchNum:" << nrx->mHeadSearchNum << " mHeadSearchTime:" << nrx->mHeadSearchTime << " mHeadSearchConstr:" << nrx->mHeadSearchConstr << endl;
		cout << "mHeadSearchMaster:" << nrx->mHeadSearchMaster << " mHeadSearchSlave1:" << nrx->mHeadSearchSlave1 << " mHeadSearchSlave2:" << nrx->mHeadSearchSlave2 << " mHeadSearchSlave3:" << nrx->mHeadSearchSlave3 << endl;
		cout << "mOptionHeading:" << nrx->mOptionHeading << endl;
		cout << "mHeadingAcc:" << nrx->mHeadingAcc << endl;
		cout << "mInnHeading:" << nrx->mInnHeading << endl;
		//filtering RTK status
		cout << "nrx->mIsGpsPosModeValid: " << nrx->mIsGpsPosModeValid << endl;
		cout << "nrx->mGpsPosMode : " << (int)nrx->mGpsPosMode << endl;
		cout << "nrx->mGpsNumObs : " << nrx->mGpsNumObs << endl;

		int finalrtkstatus = 0;
		// prevent the sudden switch to rtk
		//if (0 == rtkstatusnumber && navinfo.mHPOSAccuracy < 0.3 && rtkValid)
		cout <<"rtkstatusnumber = :" << rtkstatusnumber << endl;
		if (0 == rtkstatusnumber && navinfo.mHPOSAccuracy < 0.3 && rtkValid)
		{
			finalrtkstatus = 1;
			infoDetail = "Differential RTK float RTK Integer";
		}
		else
		{ //rtk lost
			if (navinfo.mHPOSAccuracy < 0.3 && rtkValid)
			{ //prevent the sudden switch to rtk
				rtkstatusnumber--;
			}
			else
			{
				rtkstatusnumber = WAITOUT;
			}

			finalrtkstatus = 0; //invalid
			// reckon when rtk lost
			NEDIncre2BLHIncre(navinfo.mLat, navinfo.mLon, north_meterIncre_temp, east_meterIncre_temp, reckoningLati, reckoningLongti);
			navinfo.mLat = reckoningLati;
			navinfo.mLon = reckoningLongti;

			//convert to UTM coordinates modifed by John 2017.10
			TiEV::Angle latAnglereck, lonAnglereck;
			latAnglereck.setByDegree(navinfo.mLat);
			lonAnglereck.setByDegree(navinfo.mLon);
			TiEV::WGS84Coor latLonPreckon((TiEV::LAT)latAnglereck, (TiEV::LON)lonAnglereck);
			TiEV::UTMCoor coordreckon;
			coordreckon = TiEV::latLonToUTMXY(latLonPreckon);
			navinfo.utmX = coordreckon.x;
			navinfo.utmY = coordreckon.y;

			reckoningDist += sqrt(pow(north_meterIncre_temp, 2) + pow(east_meterIncre_temp, 2));
			cout << "reckoning ... || recvd CAN speed :" << Corrected_CANspeed << endl;

			//view reckon results not reliable when reckon distance is more than 800m
			if (reckoningDist >= 800)
			{ //invalid
				navinfo.isReckoningVaild = 0;
			}
			else
			{
				navinfo.isReckoningVaild = 1;
			}
			if (rtkstatusnumber >= WAITOUT)
			{
				infoDetail = "RTK lost";
			}
			else if (navinfo.mHPOSAccuracy > 0.3)
			{
				infoDetail = "RTK precision > 0.3";
			}
			else
			{
				infoDetail = "RTK lost and RTK precision > 0.3";
			}
		}

		if (start_pts > 0)
		{
			reckoncheck1.x = navinfo.utmX;
			reckoncheck1.y = navinfo.utmY;
			start_pts--;
		}
		else
		{
			reckoncheck1.x += east_meterIncre_temp;
			reckoncheck1.y += north_meterIncre_temp;
		}

		// trans rear axle to front axle
		float axleDisOfCar = 2.3; //m
		double dn = axleDisOfCar * sin(navinfo.mHeading);
		double de = axleDisOfCar * cos(navinfo.mHeading);
		double frontaAxleUtmX = navinfo.utmX + de;
		double frontaAxleUtmY = navinfo.utmY + dn;

		TiEV::WGS84Coor latLonUpdate;
		latLonUpdate = TiEV::UTMXYToLatLon(TiEV::UTMCoor(frontaAxleUtmX, frontaAxleUtmY));
		double frontaAxleLat = latLonUpdate.lat.getDegree();
		double frontaAxleLont = latLonUpdate.lon.getDegree();

		structNAVINFO navinfoFrontAxle = navinfo;
		navinfoFrontAxle.utmX = frontaAxleUtmX;
		navinfoFrontAxle.utmY = frontaAxleUtmY;
		navinfoFrontAxle.mLat = frontaAxleLat;
		navinfoFrontAxle.mLon = frontaAxleLont;
		navinfoFrontAxle.mRTKStatus = finalrtkstatus;

		reckoncheckfile1 << fixed << reckoncheck1.x << " " << reckoncheck1.y << endl;
		rtkutm << fixed << navinfoFrontAxle.utmX << " " << navinfoFrontAxle.utmY << endl;

		zcm_publish.publish("NAVINFO", &navinfoFrontAxle);

		cout << "--------------State:" << infoDetail << "------------" << endl;
		printf("\rpkts: %d RTK: %d Accuracy: %f Lon:%3.12f Lat:%3.12f Heading:%3.4f Speed:%2.2f Pitch:%2.4f\n", nrx->mCmdPkts,
			   finalrtkstatus, navinfoFrontAxle.mHPOSAccuracy, navinfoFrontAxle.mLon, navinfoFrontAxle.mLat, TiEV::rad2deg(navinfoFrontAxle.mHeading), navinfoFrontAxle.mSpeed3d, navinfoFrontAxle.mPitch);
		if (bReport)
		{
			time(&now);
			timenow = localtime(&now);
			fprintf(fp_pos, "%d-%d-%d %3.12f %3.12f %4.2f %3.4f %3.4f %3.4f %2.2f\n", timenow->tm_hour, timenow->tm_min,
					timenow->tm_sec, navinfoFrontAxle.mLon, navinfoFrontAxle.mLat, navinfoFrontAxle.mAlt, navinfoFrontAxle.mHeading, navinfoFrontAxle.mCurvature,
					navinfoFrontAxle.mAngularRateZ, navinfoFrontAxle.mSpeed3d);
		}

		mtx.unlock();
		usleep(20000);
	}
	reckoncheckfile1.close();
	rtkutm.close();
	// speedcheck.close();
	if (bReport)
	{
		fclose(fp_pos);
	}
}

//debug
void listenfunc_file(NComRxC *nrx, bool bReport)
{
	if (nrx == NULL)
	{
		printf("NComRxC error!\n");
		return;
	}
	try
	{
		char buff[SIZE_BUF];
		FILE *fp_raw;
		fp_raw = fopen("C:\\Users\\JohnZhao\\Desktop\\IMUdebug\\map4_10s.ncom", "rb");

		while (1)
		{
			mtx.lock();
			if (fread(buff, sizeof(char), SIZE_BUF, fp_raw) != SIZE_BUF)
			{
				break;
			}
			//Decode the package
			NComNewChars(nrx, (unsigned char *)&buff, SIZE_BUF);
			mtx.unlock();
			usleep(0); //Sleep(1) cause serious delay
		}
		fclose(fp_raw);
	}
	catch (std::exception &ex)
	{
		std::cout << ex.what();
	}
}

void recvzcm(zcm::ZCM &zcmer)
{
	while (0 == zcmer.handle())
		;
}

int main(int argc, char *argv[])
{
	bool bReport = true;
	if (argc == 1)
	{
		printf("Do not record pos\n");
		//	bReport = false;
	}
	else if (argc == 2 && !strcmp(argv[1], "-r"))
	{
		printf("Record pos\n");
		bReport = true;
	}
	else
	{
		printf("WRONG Parameter! Usage:IMU_receiver.exe or IMU_receiver.exe -r, the latter will save the reported data in a txt file.\n");
		return 0;
	}

	//used for report pos file
	if (access("../log", 6) == -1)
	{
		//if the log fold does not exist, create one
		mkdir("../log", S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH);
	}
	time(&now);
	timenow = localtime(&now);
	sprintf(Buffer_pos_path,
			"../log/%d-%02d-%02d_%02d_%02d_%02d",
			timenow->tm_year,
			timenow->tm_mon,
			timenow->tm_mday,
			timenow->tm_hour,
			timenow->tm_min,
			timenow->tm_sec);

	zcm::ZCM zcmer{};
	if (!zcmer.good())
	{
		cout << "zcm init error!" << endl;
		system("pause");
	}

	Handler handlerObject;
	zcmer.subscribe("CANINFO", &Handler::handleMessage, &handlerObject);

	IMU_RECEIVER receiver;
	std::thread thread1(listenfunc, receiver.get_ncom(), bReport);
	std::thread thread2(reportfunc, receiver.get_ncom(), bReport, ref(zcmer));
	std::thread thread3(recvzcm, ref(zcmer));

	thread1.join();
	thread2.join();
	thread3.join();

	return 0;
}
