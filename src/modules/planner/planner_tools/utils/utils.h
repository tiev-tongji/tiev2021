#ifndef _UTILS_H_
#define _UTILS_H_

#include "nature/point.h"
#include <math.h>

namespace TiEV{
namespace utils{
using namespace std;

//return rad of angle from start point to end point
double getAngleOfAlongDirection(Point &start_point, Point &end_point);

double getAngleOfNormalDirection(Point &start_point, Point &end_point);

//params: start point, the lateral distance, + is left, - is right
Point getLateralPoint(Point &start_point, double lateral_distance);

//params: start point, the longitude distance, + is left, - is right
Point getLongitudePoint(Point &start_point, double longitude_distance);

//if point p is in our local map TiEV::GRID_ROW*TIEV::GRID_COL, return true
bool isInLocalMap(const Point &p);

void normalizeAngle(double &rad_angle);

double getLateralDistance(const Point& standard_point, const Point& p);

}//namespace utils
}//namespace TiEV

#endif
