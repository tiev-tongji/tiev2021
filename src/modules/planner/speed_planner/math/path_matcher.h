#pragma once

#include "linear_interpolation.h"
#include "nature.h"
#include <vector>

namespace TiEV {

/**
 * @class
 * @brief Math a given coordinate(X/Y) to a given path and convert it to Frenet coordinate
 */
class PathMatcher {
public:
    PathMatcher() = delete;

    /**
     * @brief Match a cartesian coordinate to a given path
     * @param path
     * @param x
     * @param y
     * @return The matched path point
     */
    static Point MatchToPath(const std::vector<Point>& path,
                                const double x, const double y);

    /**
     * @brief Match given s to a given path
     * @param path
     * @param s
     * @return The matched path point
     */
    static Point MatchToPath(const std::vector<Point>& path,
                                 const double s);

    /**
     * @brief Convert a cartesian coordinate to a Frenet coordinate
     * @param path
     * @param x
     * @param y
     * @return Frenet coordinate
     */
    static std::pair<double, double> GetPathFrenetCoordinate(
            const std::vector<Point>& path,
            const double x, const double y);

private:
    /**
     * @brief Find the projection point of given point on the given path
     *        using linear interpolation
     * @param p0 The first path point
     * @param p1 The second path point
     * @param x
     * @param y
     * @return The projection point
     */
    static Point FindProjectionPoint(const Point& p0, const Point& p1,
                                         const double x, const double y);
};

} // namespace TiEV