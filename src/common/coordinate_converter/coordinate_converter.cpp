#include "coordinate_converter.h"
#include "basic_coordinate_converter.h"
#include "nature/angle.h"

#include <iomanip>

using namespace std;

namespace TiEV{

LAT::LAT(const Angle & lat) : Angle(lat) {}
LAT::LAT() {}

double LAT::getValue() const {
    return Angle::getRad();
}


LON::LON(const Angle & lon) : Angle(lon) {}
LON::LON() {}

double LON::getValue() const {
    return Angle::getRad();
}

UTMCoor::UTMCoor(const UTMCoor & T): x(T.x), y(T.y) {}
UTMCoor::UTMCoor(const double _x, const double _y): x(_x), y(_y) {}
UTMCoor::UTMCoor(): x(0), y(0) {}

WGS84Coor::WGS84Coor(LAT _lat, LON _lon):lat(_lat), lon(_lon) {}
WGS84Coor::WGS84Coor(){}

UTMCoor latLonToUTMXY(const WGS84Coor & latLonCoor){
    THIRD_PARTY_LIB::UTMCoor tmp = THIRD_PARTY_LIB::LatLonToUTMXY(latLonCoor.lat.getValue(), latLonCoor.lon.getValue(), 51);
    UTMCoor res(tmp.x, tmp.y);
    return res;
}

WGS84Coor UTMXYToLatLon(const UTMCoor & UTMCoor, int zone, bool southhemi){
    THIRD_PARTY_LIB::WGS84Coor tmp = THIRD_PARTY_LIB::UTMXYToLatLon(UTMCoor.x, UTMCoor.y, zone, southhemi);
    WGS84Coor res;
    Angle lat, lon;
    lat.setByRad(tmp.lat);
    lon.setByRad(tmp.lon);
	res.lat = (LAT)lat;
    res.lon = (LON)lon;
    return res;
}

}
