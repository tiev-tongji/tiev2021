#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "const.h"
#include "opencv2/opencv.hpp"
#include "point2d.h"
#include "pose.h"
using std::vector;

namespace TiEV {

class PathSmoother {
 public:
  PathSmoother(double learning_rate, double max_iteration, double weight_smooth,
               double weight_curvature, double weight_obstacle)
      : alpha(learning_rate),
        maxIteration(max_iteration),
        wSmoothness(weight_smooth),
        wCurvature(weight_curvature),
        wObstacle(weight_obstacle),
        kappaMax(0.1818),
        obsDMax(10),
        width(MAX_ROW),
        height(MAX_COL) {}
  // api
  vector<Point2d> smoothPath(const vector<Point2d>& path, const int gap = 5);

  // smoothPath for lattice planner
  vector<Point2d> smoothPath(const vector<HDMapPoint>& path, const int gap,
                             std::string type);

  template <typename T>
  vector<Point2d> convert2Point2d(const vector<T>& original_path);

  // linear interpolate n points between original path points
  vector<Point2d> interpolatePath(const vector<Point2d>& path, const int n);

  /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring
  /// drivability
  // xim1 means x_(i-1), xip1 means x_(i+1)
  Point2d curvatureTerm(const Point2d& xim1, const Point2d& xi,
                        const Point2d& xip1);

  /// smoothnessCost - attempts to spread nodes equidistantly and with the same
  /// orientation
  Point2d smoothnessTerm(const Point2d& xim2, const Point2d& xim1,
                         const Point2d& xi, const Point2d& xip1,
                         const Point2d& xip2);

  double getCurvature(const Point2d& xim1, const Point2d& xi,
                      const Point2d& xip1);

  double getTotalCost(const vector<Point2d>& path);

  /// a boolean test, whether vector is on the grid or not
  bool isOnGrid(Point2d vec) {
    if (vec.x >= 0 && vec.x < width && vec.y >= 0 && vec.y < height) {
      return true;
    }
    return false;
  }

  template <class T>
  T clamp(const T& value, const T& lb, const T& ub);

 private:
  /// maximum possible curvature of the non-holonomic vehicle
  double kappaMax;  // unit 1/m
  /// maximum distance to obstacles that is penalized
  double obsDMax;  // unit obstacles that is penalized

  double alpha;
  /// weight for the obstacle term
  double wObstacle;
  /// weight for the curvature term
  double wCurvature;
  /// weight for the smoothness term
  double wSmoothness;
  /// max_iteration;
  int maxIteration;

  /// width of the map
  int width;
  /// height of the map
  int height;
};
}  // namespace TiEV
#endif  // SMOOTHER_H