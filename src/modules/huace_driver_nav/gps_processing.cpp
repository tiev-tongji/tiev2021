/**
 * @file gps_processing.cpp
 * @author Jiafeng Cui
 * @brief
 * @version 0.1
 * @date 4/10/20
 * @copyright Copyright (c) 2020 All rights reserved.
 *
 */

#include <vector>
#include <sstream>
#include <iostream>
#include "gps_processing.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
const unsigned char  HEX_TO_ASCII[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
/**$GPCHC,2131,265126.20,0.00,-1.33,-0.14,0.02,0.21,-0.01,0.0024,-0.0232,1.0005,0.00000000,0.00000000,0.00,0.000,0.000,0.000,0.000,0,0,01,0,2*5C
 * $GPCHC,2131,265130.00,0.00,-1.28,-0.17,0.06,0.20,-0.06,0.0030,-0.0224,1.0003,0.00000000,0.00000000,0.00,0.000,0.000,0.000,0.000,0,0,01,0,2*56
 status: 高位->4 || 5 :RTK GOOD; 低位: 1->gps;2->gps+imu;3->imu;0->null
 warning: 00_00 正常
 LONGITUDE -180~+180
 LATITUDE  -90~+90
KNOTS_TO_KM 1.852  1海里 = 1.852千米
*/
template <typename T>
void StringConverter(std::string &str, T &num) {
    std::stringstream ss;
    ss << str;
    ss >> num;
}

void VectorizeString(
        const std::string& str,
        std::vector<std::string>& vectorized_message,
        const std::string& delimiters)
{
    boost::algorithm::split(vectorized_message, str, boost::algorithm::is_any_of(delimiters));
}

/**
 * 异或校验
 * @param s
 * @return
 */
int  CheckGpsData( std::string s )
{
    unsigned char  crc8 = 0;
    int i = 0 ;
    while( s[i] != '$' )
    {
        if( ( s[i] == '\0' ) || ( s[i] == '*' ) ) return 0;

        i++;
    }

    i++;
    crc8 = s[i++];

    while( s[i] != '*' )
    {
        if( s[i] == '\0' ) return 0;

        crc8 ^= s[i++];
    }

    i++;

    if( HEX_TO_ASCII[( crc8 >> 4 ) & 0x0f] == s[i++] )
    {
        if( HEX_TO_ASCII[crc8 & 0x0f] == s[i] )
        {
            return 1;
        }
    }

    return 0;
}


int GpsData::GpsProcessing(std::string buff_string,GpsData &gps) {
    if (CheckGpsData(buff_string)) { /// valid data
        std::vector<std::string> buff_temp;
        VectorizeString(buff_string, buff_temp, ",");
        if (buff_temp[0] == "$GPCHC") {
            if (buff_temp.size() != 24) {
                std::cout << "this sentence has not enough 24 strings!!!!!!!!!!!!!!!!!!!!!!";
                return 0;
            }
            else { /// 24 strings
                if (atof(buff_temp[3].c_str()) >= 0 and
                    atof(buff_temp[3].c_str()) <= 360) { /// heading角在0~360degree,认为有效
                    StringConverter(buff_temp[1], gps.week);
                    StringConverter(buff_temp[2], gps.time);
                    StringConverter(buff_temp[3], gps.heading);
                    StringConverter(buff_temp[4], gps.pitch);
                    StringConverter(buff_temp[5], gps.roll);
                    StringConverter(buff_temp[6], gps.angular_x);
                    StringConverter(buff_temp[7], gps.angular_y);
                    StringConverter(buff_temp[8], gps.angular_z);
                    StringConverter(buff_temp[9], gps.acceleration_x);
                    StringConverter(buff_temp[10], gps.acceleration_y);
                    StringConverter(buff_temp[11], gps.acceleration_z);
                    StringConverter(buff_temp[12], gps.latitude);
                    StringConverter(buff_temp[13], gps.longitude);
                    StringConverter(buff_temp[14], gps.altitude);
                    StringConverter(buff_temp[15], gps.speed_east);
                    StringConverter(buff_temp[16], gps.speed_north);
                    StringConverter(buff_temp[18], gps.speed);
                    StringConverter(buff_temp[19], gps.num_satellites);
                    StringConverter(buff_temp[21], gps.status);
                    // StringConverter(buff_temp[23],gps.warning);
                    return 1;
                } else { /// data is wrong
                    std::cout << "although have 24 strings, but data is wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
                    return 0;
                }
            }
        } else {
            std::cout << "gps data format is wrong!" << "\n";
            return 0;
        }
    }else{
        return 0;
    }
}





