#pragma once

#include "linear_interpolation.h"
#include "../../../include/planner_common/pose.h"
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
    static Pose MatchToPath(const std::vector<Pose>& path,
                                const double x, const double y);

    /**
     * @brief Match given s to a given path
     * @param path
     * @param s
     * @return The matched path point
     */
    static Pose MatchToPath(const std::vector<Pose>& path,
                                 const double s);

    /**
     * @brief Convert a cartesian coordinate to a Frenet coordinate
     * @param path
     * @param x
     * @param y
     * @return Frenet coordinate
     */
    static std::pair<double, double> GetPathFrenetCoordinate(
            const std::vector<Pose>& path,
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
    static Pose FindProjectionPoint(const Pose& p0, const Pose& p1,
                                         const double x, const double y);
};

} // namespace TiEV