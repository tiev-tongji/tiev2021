#ifndef _POINT_H_
#define _POINT_H_

#include <cmath>
#include <cstring>
#include <string>
#include <sstream>
#include "coordinate_converter/coordinate_converter.h"
#include "angle.h"
#include "nature.h"
using namespace std;

namespace TiEV{

class Point{
public:
    //Global coordinate
    double lat = -1;
    double lon = -1;
    double utmX = -1;
    double utmY = -1;
    TiEV::Angle heading;    //0 is east, decrease clockwise

    //Local coordinate
    double x = -1;  // 0.2m
    double y = -1;  // 0.2m
    TiEV::Angle angle;  //0 is the direction of the car, increase clockwise

    //Path message
	double a = 0;   // m/(s^2)
    double v = 0;  // m/s
    double t = 0;  // s
    double k = 0;   // 1/m
    double s = 0;  // m
    bool backward = false;

    //Get a point with lat, lon, utmx and utmy
    static Point fromLatLon(double lat, double lon){
        Point res;
        Angle a, o;
        a.setByDegree(lat);
        o.setByDegree(lon);
        UTMCoor utmxy = latLonToUTMXY(WGS84Coor(LAT(a), LON(o)));
        res.lat = lat;
        res.lon = lon;
        res.utmX = utmxy.x;
        res.utmY = utmxy.y;
        return res;
    }

    //Get a point with lat, lon, utmx and utmy
    static Point fromUTMXY(double utmx, double utmy){
        Point res;
        WGS84Coor latlon = UTMXYToLatLon(UTMCoor(utmx, utmy));
        res.lat = latlon.lat.getDegree();
        res.lon = latlon.lon.getDegree();
        res.utmX = utmx;
        res.utmY = utmy;
        return res;
    }

    /* Use a standard point with the correct local coordinate and global coordinate values to update x, y and angle of this point.
     * This point should have correct global coordinate.
     */
    void updateLocalCoordinate(const Point& standard_point){
        double stdh = standard_point.heading.getRad() - standard_point.angle.getRad();
        double qx = (utmX - standard_point.utmX) / GRID_RESOLUTION;
        double qy = (utmY - standard_point.utmY) / GRID_RESOLUTION;
        double sinstdh = sin(stdh), cosstdh = cos(stdh);
        double px = (qx * cosstdh + qy * sinstdh);
        double py = (-qx * sinstdh + qy * cosstdh);
        x = standard_point.x - px;
        y = standard_point.y - py;

        double ag = standard_point.heading.getRad() +
            standard_point.angle.getRad() - heading.getRad();
        while(ag < -Angle::PI) ag += 2 * Angle::PI;
        while(ag >= Angle::PI) ag -= 2 * Angle::PI;
        angle.setByRad(ag);
    }

    /* Use a standard point with the correct local coordinate and global coordinate values to update utmx, utmy, lat, lon and heading of this point.
     * This point should have correct local coordinate.
     */
    void updateGlobalCoordinate(const Point& standard_point){
        double stdh = standard_point.heading.getRad() - standard_point.angle.getRad();
        double sinstdh = sin(stdh), cosstdh = cos(stdh);
        double px = standard_point.x - x;
        double py = standard_point.y - y;
        double qx = (px * cosstdh - py * sinstdh);
        double qy = (px * sinstdh + py * cosstdh);
        utmX = qx * GRID_RESOLUTION + standard_point.utmX;
        utmY = qy * GRID_RESOLUTION + standard_point.utmY;
        Point p = Point::fromUTMXY(utmX, utmY);
        lat = p.lat;
        lon = p.lon;

        double hd = standard_point.heading.getRad() +
            standard_point.angle.getRad() - angle.getRad();
        while(hd < -Angle::PI) hd += 2 * Angle::PI;
        while(hd >= Angle::PI) hd -= 2 * Angle::PI;
        heading.setByRad(hd);
    }

    string to_string() const{
        std::stringstream stream;
        stream << "lat,lon: (" << lat << "," << lon << ")" << endl
            << "heading: " << heading.getDegree() << "D" << endl
            << "utm: (" << utmX << "," << utmY << ")" << endl
            << "x,y: (" << x << "," << y << ")" << endl
            << "angle: " << angle.getDegree() << "D" << endl
            << "v,t,k,s: (" << v << "," << t << "," << k << "," << s <<")";
        return stream.str();
    }
};

}

#endif
