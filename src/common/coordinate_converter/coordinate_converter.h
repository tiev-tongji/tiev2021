//TODO : add proclaim
//		add Lat Lon class
//		translate function interface
//

#ifndef _COORDINATE_CONVERTER_H_
#define _COORDINATE_CONVERTER_H_

#include "nature/angle.h"

namespace TiEV{

struct LAT : public Angle{
    explicit LAT();
    explicit LAT(const Angle & lat);
    double getValue() const;

};

struct LON : public Angle{
    explicit LON();
    explicit LON(const Angle & lon);
    double getValue() const;

};

struct UTMCoor{
    double x;
    double y;

    UTMCoor(const UTMCoor & T);
    UTMCoor(const double _x, const double _y);
    UTMCoor();
};

struct WGS84Coor
{
    LAT lat;
    LON lon;
    WGS84Coor(LAT _lat, LON _lon);
    WGS84Coor();
};

UTMCoor latLonToUTMXY(const WGS84Coor & latLonCoor);
WGS84Coor UTMXYToLatLon(const UTMCoor & UTMCoor, int zone = 51, bool southhemi = 0);

}
#endif //_PLANNER_SRC_COORDINATECONVERTER_H_
