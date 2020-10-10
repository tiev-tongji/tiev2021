#include "path_matcher.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace TiEV {

Point PathMatcher::MatchToPath(const std::vector<Point> &path,
                                  const double x, const double y) {
    auto func_dis_square = [](const Point& point, const double x, const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return dx * dx + dy * dy;
    };

    double dis_min = func_dis_square(path.front(), x, y);
    std::size_t idx_min = 0;

    for (std::size_t i = 1; i < path.size(); i ++) {
        double dis_tmp = func_dis_square(path[i], x, y);
        if (dis_tmp < dis_min) {
            dis_min = dis_tmp;
            idx_min = i;
        }
    }

    std::size_t idx_start = (idx_min == 0) ? idx_min : idx_min - 1;
    std::size_t idx_end = (idx_min + 1 == path.size()) ? idx_min : idx_min + 1;

    if (idx_start == idx_end) {
        return path[idx_start];
    }

    return FindProjectionPoint(path[idx_start], path[idx_end], x, y);
}

Point PathMatcher::MatchToPath(const std::vector<Point> &path,
                                   const double s) {
    auto comp = [](const Point& point, const double s) {
        return point.s < s;
    };

    auto it_lower = std::lower_bound(path.begin(), path.end(), s, comp);
    if (it_lower == path.begin()) return path.front();
    else if (it_lower == path.end()) return path.back();

    return InterpolateUsingLinearApproximationWithS(*(it_lower - 1), *it_lower, s);
}

Point PathMatcher::FindProjectionPoint(const Point &p0, const Point &p1,
                                           const double x, const double y) {
     double v0x = (x - p0.x) * GRID_RESOLUTION;
     double v0y = (y - p0.y) * GRID_RESOLUTION;

     double v1x = (p1.x - p0.x) * GRID_RESOLUTION;
     double v1y = (p1.y - p0.y) * GRID_RESOLUTION;

     double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
     double dot = v0x * v1x + v0y * v1y;

     double delta_s = dot / v1_norm;
     return InterpolateUsingLinearApproximationWithS(p0, p1, p0.s + delta_s);
}

std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(const std::vector<Point> &path, const double x,
                                                               const double y) {
    Point matched_path_point = MatchToPath(path, x, y);
    double rtheta = matched_path_point.angle.getRad();
    double rx = matched_path_point.x;
    double ry = matched_path_point.y;
    double delta_x = x - rx;
    double delta_y = y - ry;

    // The original coord represents gridded cells
    // Here we need to transfer it to actual distance
    double dis_x = delta_x * GRID_RESOLUTION;
    double dis_y = delta_y * GRID_RESOLUTION;

    // Check the sign of lateral value using vector crossing
    double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
    std::pair<double, double> coord;
    coord.first = matched_path_point.s;
    coord.second = std::copysign(std::hypot(dis_x, dis_y), side);
    return coord;
}
} // namespace TiEV
