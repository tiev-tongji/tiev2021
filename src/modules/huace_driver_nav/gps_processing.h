/**
 * @file gps_processing.h
 * @author Jiafeng Cui
 * @brief
 * @version 0.1
 * @date 4/10/20
 * @copyright Copyright (c) 2020 All rights reserved.
 *
 */

#ifndef GPS_PROCESSING_H
#define GPS_PROCESSING_H

    class GpsData {
    public:
        GpsData():
                latitude(0),
                longitude(0),
                altitude(0),
                speed(0),
                speed_east(0),
                speed_north(0),
                acceleration_x(0),
                acceleration_y(0),
                acceleration_z(0),
                angular_x(0),
                angular_y(0),
                angular_z(0),
                roll(0),
                pitch(0),
                week(0),
                time(0),
                heading(0),
                status(0),
                num_satellites(0),
                warning(0){}
    int GpsProcessing(std::string buff_string,GpsData &gps);


    public:
        uint16_t week;
        float time;
        double heading;
        float pitch;
        float roll;
        float angular_x;
        float angular_y;
        float angular_z;
        double acceleration_x;
        double acceleration_y;
        double acceleration_z;
        double latitude;
        double longitude;
        double altitude;
        float speed_east;
        float speed_north;
        float speed;
        int status;
        uint8_t num_satellites;
        uint8_t warning;
    };



#endif //GPS_PROCESSING_H
