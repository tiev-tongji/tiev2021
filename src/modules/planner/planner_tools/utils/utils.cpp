#include "utils.h"

namespace TiEV{
namespace utils{

using namespace std;

//return rad of angle from start point to end point
double getAngleOfAlongDirection(Point &start_point, Point &end_point){
	double heading = atan2(end_point.y - start_point.y, end_point.x - start_point.x);
	heading = Angle::PI - heading;
	while(heading > Angle::PI) heading -= 2 * Angle::PI;
	while(heading <= -Angle::PI) heading += 2 * Angle::PI;
	return heading;
}

void normalizeAngle(double &rad_angle){
	while(rad_angle > Angle::PI) rad_angle -= 2 * Angle::PI;
	while(rad_angle <= -Angle::PI) rad_angle += 2 * Angle::PI;
}

double getAngleOfNormalDirection(Point &start_point, Point &end_point){
	double heading = getAngleOfAlongDirection(start_point, end_point);
	heading += Angle::PI / 2;
	while(heading > Angle::PI) heading -= 2 * Angle::PI;
	while(heading <= -Angle::PI) heading += 2 * Angle::PI;
	return heading;
}

//params: start point, the lateral distance, + is left, - is right, disatance unit (m)
Point getLateralPoint(Point &start_point, double lateral_distance){
	Point res_point = start_point;
	double angle = start_point.angle.getRad();
	angle = Angle::PI / 2 - angle;
	while(angle > Angle::PI) angle -= 2 * Angle::PI;
	while(angle <= -Angle::PI) angle += 2 * Angle::PI;
	double cos_ang = cos(angle);
	double sin_ang = sin(angle);
	res_point.x = start_point.x - lateral_distance / GRID_RESOLUTION * cos_ang;
	res_point.y = start_point.y - lateral_distance / GRID_RESOLUTION * sin_ang;
	return res_point;
}

//params: start point, the logitude distance, + is forward, - is bcakward, distance unit (m)
Point getLongitudePoint(Point &start_point, double longitude_distance){
	Point res_point = start_point;
	double angle = start_point.angle.getRad();
	angle = Angle::PI - angle;
	while(angle > Angle::PI) angle -= 2 * Angle::PI;
	while(angle <= -Angle::PI) angle += 2 * Angle::PI;
	double cos_ang = cos(angle);
	double sin_ang = sin(angle);
	res_point.x = start_point.x + longitude_distance / GRID_RESOLUTION * cos_ang;
	res_point.y = start_point.y + longitude_distance / GRID_RESOLUTION * sin_ang;
	return res_point;
}

bool isInLocalMap(const Point &p){
	if(p.x >= 0 && p.y >= 0 && p.x < GRID_ROW && p.y < GRID_COL) return true;
	return false;
}

//left is +, right is - (m)
double getLateralDistance(const Point& standard_point, const Point& p){
	double theta = Angle::PI - standard_point.angle.getRad();
	normalizeAngle(theta);
	double k = tan(theta);
	double distance = (p.y - standard_point.y - k * (p.x - standard_point.x)) / sqrt(k * k + 1);
	double cos_ang = cos(standard_point.angle.getRad());
	distance = -distance * (cos_ang / fabs(cos_ang)) * GRID_RESOLUTION;
	return distance;
}

}
}
