#ifndef _UNIT_H_
#define _UNIT_H_

#include <cmath>

//#define TiEV_PI 3.141592653589793238462643383279502884197169

namespace TiEV{
const double TiEV_PI = 3.141592653589793238462643383279502884197169;

/*
 * kph2ms
 *
 * Converts kilometers per hour to meters per second.
 *
 */
inline double kph2ms(double kph) {
    return (kph * 0.277777778);
}
/*
 * ms2kph
 *
 * Converts meters per second to kilometers per hour.
 *
 */
inline double ms2kph(double ms) {
    return (ms * 3.6);
}

/*
 * deg2rad
 *
 * Converts degrees to radians.
 *
 */
inline double deg2rad(double deg)
{
    return (deg / 180.0 * TiEV_PI);
}

/*
 * rad2deg
 *
 * Converts radians to degrees.
 *
 */
inline double rad2deg(double rad)
{
    return (rad / TiEV_PI * 180.0);
}

/*
 * rotate2d
 *
 * rotate (x, y) based on rad, in counterclockwise direction
 *
 */
inline void rotate2d(double &x, double &y, const double rad)
{
    double cosh = cos(rad);
    double sinh = sin(rad);
    double tx = cosh * x - sinh * y;
    double ty = sinh * x + cosh * y;

    x = tx;
    y = ty;
}
/*
 * translate2d
 *
 * translate (x, y) based on （dx, dy）
 *
 */
inline void translate2d(double &x, double &y, const double dx, const double dy)
{
    x = x + dx;
    y = y + dy;
}

/*
 * normalize_theta
 *
 * normalize a theta (in Rad) to -pi ~ pi
 *
 */
inline double normalize_theta(double theta) {
    double multiplier;

    if (theta >= -TiEV_PI && theta < TiEV_PI) return theta;

    multiplier = floor(theta / (2 * TiEV_PI));
    theta = theta - multiplier * 2 * TiEV_PI;
    if (theta >= TiEV_PI) theta -= 2 * TiEV_PI;
    if (theta < -TiEV_PI) theta += 2 * TiEV_PI;

    return theta;
}

/*
 * ned_deg2iso_rad
 *
 * north east down to east north up (-pi ~ pi)
 *
 */
inline double ned_deg2iso_rad(double degree)
{
    return normalize_theta(deg2rad(-1 * degree) + deg2rad(90.));
}

/*
 * ned_deg2iso_rad
 *
 * north east down to east north up (-180 ~ 180)
 *
 */
inline double ned_deg2iso_deg(double degree)
{
    return rad2deg(normalize_theta(deg2rad(-1 * degree) + deg2rad(90.)));
}

}

#endif // _UNIT_H_
