#ifndef DGC_TRACKER_H
#define DGC_TRACKER_H

#include "perception.h"
#include <utils.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

void bounding_box(std::vector<point3d_t>* points, float angle,
                  double* x, double* y, double *yaw, double* width, double* length,
                  double min_car_width, double min_car_length);

void bounding_box(TiEV::Obstacle* obstacle, double angle, double min_car_width, double min_car_length);

// fit a car-shaped bounding box to an obstacle with optimal alignment
void align_bounding_box(TiEV::Obstacle* obstacle, double min_car_width, double min_car_length);

float align_obstacle(TiEV::Obstacle* obstacle);

// return principal alignment as calculated by 1D Hough transform with 90 degree rotational invariance
float align_points(std::vector<point3d_t>& points);


#endif
