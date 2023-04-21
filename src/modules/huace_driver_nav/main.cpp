#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include "unistd.h"
#include "gps_processing.h"
#include "serial.h"
#include <zcm/zcm.h>
#include "zcm/zcm-cpp.hpp"
#include "MsgNavInfoSignal.hpp"
#include "tievmsg.h"
#include "MsgRTK.hpp"
#include "WGS84UTM.h"
#include "nature.h"
#include "coordinate_converter/basic_coordinate_converter.h"
constexpr uint32_t GpsAndSystemDiffSeconds = 315964800; // 1980-01-06 vs 1970-01-01
constexpr uint32_t SecondsPerWeek = 60 * 60 * 24 * 7;
const std::string GPCHC_BEGIN_CHAR = "$";
const std::string GPCHC_ENDLINE = "\r\n"; // 结束符
using namespace std;
// zcm::ZCM pubzcm{"udpm://239.255.76.67:7667?ttl=1"};
zcm::ZCM pubzcm{"ipc"};


int main(int argc, char **argv) {
//        string path1 = "./data_receiver.txt";
//        string path2 = "./buff_string.txt";
//        string path3 = "./gps_processing.txt";
//        int len_total;

    GpsData gps;
    serial::Serial ser; //声明串口对象
    try {
        ser.setPort("/dev/huace");
        ser.setBaudrate(460800);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 1second
        ser.setTimeout(to);
        ser.close();
        ser.open();
    }
    catch (serial::IOException &e) {
        cerr << "Unable to open port " << "\n";
        return -1;
    }
    // 检测串口是否已经打开，并给出提示信息
    if (ser.isOpen()) {
        cout << "Serial Port initialized" << "\n";
    } else {
        cerr << "[FATAL]~~~~~~~~Serial Port uninitialized" << "\n";
        return -1;
    }
    std::string buff_string;
    std::string buffer;
    size_t max_size = 65536;
    structNAVINFO msg_nav_temp;
    while (pubzcm.good()) {
        while (ser.available()) {
            // cout << "Reading from serial port" << "\n";
            buffer = ser.readline(max_size,GPCHC_ENDLINE);
            buff_string += buffer;
            int index = 0, start_index = -1, end_index = -1;
            while (index < buff_string.length()) {
                start_index = buff_string.find(GPCHC_BEGIN_CHAR);
                if (start_index == -1) {
                    cout << "datas in this time is useless" << "\n";
                    buff_string.clear();
                    break;
                } // 没有起始位,常出现于刚开始,丢弃所有数据
                else // 找到起始位
                {
                    end_index = buff_string.find(GPCHC_ENDLINE);
                    if (end_index == -1) { // 有起始位没有结束位
                        buff_string = buff_string.substr(start_index); // 留下后半段
                        break;
                    } else { // 有起始位也有结束位
                        index = end_index;
                        int valid_gps_data = gps.GpsProcessing(
                                buff_string.substr(start_index, end_index - start_index + 2), gps);
                        if (valid_gps_data > 0) // 正确解析数据
                        {
                            //int gps_status_high, gps_status_low;
                            msg_nav_temp.timestamp = ((static_cast<int64_t>(GpsAndSystemDiffSeconds + gps.week * SecondsPerWeek) - 18) * static_cast<int64_t>(100) +
                                   			static_cast<int64_t>(gps.time * 100.0 + 0.5) ) * static_cast<int64_t>(10000);
                            msg_nav_temp.mLat = gps.latitude;
                            msg_nav_temp.mLon = gps.longitude;
                            msg_nav_temp.mAlt = gps.altitude;

   			    TiEV::Angle angle_lon, angle_lat;
    			    angle_lon.setByDegree(msg_nav_temp.mLon);
    			    angle_lat.setByDegree(msg_nav_temp.mLat);
    			    TiEV::WGS84Coor latlon_point((TiEV::LAT)angle_lat, (TiEV::LON)angle_lon);
    			    TiEV::UTMCoor utm_point = TiEV::latLonToUTMXY(latlon_point);

                          //  THIRD_PARTY_LIB::UTMCoor xy = THIRD_PARTY_LIB::LatLonToUTMXY(msg_nav_temp.mLat,msg_nav_temp.mLon,51);
                            msg_nav_temp.utmX =utm_point.x ;
                            msg_nav_temp.utmY =utm_point.y;

                            msg_nav_temp.mSpeed3d = gps.speed;
                            msg_nav_temp.mVe = gps.speed_east;
                            msg_nav_temp.mVn = gps.speed_north;
                            msg_nav_temp.mAx = gps.acceleration_x * 9.7964;
                            msg_nav_temp.mAy = gps.acceleration_y * 9.7964;
			    if(gps.heading > 270 && gps.heading < 360){
				msg_nav_temp.mHeading = (450 - gps.heading) / 180 * pi;
			    }
			    else{
                            	msg_nav_temp.mHeading = (90 - gps.heading) / 180 * pi;
			    }
                            msg_nav_temp.mPitch = gps.pitch / 180 * pi;
                            // msg_nav_temp.mRoll = gps.roll / 180 * pi;
                            msg_nav_temp.mAngularRateZ = gps.angular_z / 180 * pi;
                            msg_nav_temp.mCurvature = 0;
                            msg_nav_temp.mGpsNumObs = gps.num_satellites;
                            msg_nav_temp.mHPOSAccuracy = 0;
                        //    gps_status_high = gps.status / 10;
                            std::cout << "GPS Heaing_deg:" << gps.heading << std::endl;
                            std::cout << "Nav Heaing_deg:" << msg_nav_temp.mHeading / pi * 180 << std::endl;
                            std::cout<<"x-aixs acc: "<< msg_nav_temp.mAx <<" || y-aixs acc: "<< msg_nav_temp.mAy<<" || z-aixs acc: "<< gps.acceleration_z * 9.7964 <<endl;
                           // gps_status_low = gps.status - 10 * gps_status_high;
                            if (gps.status == 42 || gps.status == 82) {
                                msg_nav_temp.mRTKStatus = 1;
                                msg_nav_temp.isReckoningVaild = 1;
                            } else {   
                                msg_nav_temp.mRTKStatus = 0;
                                msg_nav_temp.isReckoningVaild = 0;
                            }
			 std::cout <<"RTK status is: "<<(int)msg_nav_temp.mRTKStatus<<","<<gps.status <<"\n";
            //  std::cout<<"msg_nav_temp.utm_x: "<<msg_nav_temp.utm_x<<endl;
                            pubzcm.publish("NAVINFO", &msg_nav_temp);
                        }
                        //不管解析的gps数据是否发布,都应该从缓存中剔除,进入到这个条件分支下的缓存区还是原来的状态,
                        //没有减小,如果缓存中去掉此次gps的字符串还有剩余字符,则取子串,再进入while循环判断,
                        //否则清空缓存区跳出循环.因此,不需要检查缓存的大小模块,缓存不会超标
                        if (index + 2 < buff_string.length()) {
                            buff_string = buff_string.substr(end_index + 2);
                            index = 0;
                            start_index = -1;
                            end_index = -1; // reset
                            continue;
                        } else {
                            buff_string.clear();
                            break;
                        }
                    }
                }

            }
        }
        usleep(10000);
    }
    ser.close();
    return 0;
}
