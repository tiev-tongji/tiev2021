#include "box.h"
#include <roadrunner.h>
#include <utils.h>
#include <assert.h>

using namespace std;
using namespace TiEV;
//static std::vector<std::map<float,float >> aaa;

void bounding_box(vector<point3d_t>* points, float angle,
                  double* x, double* y, double *yaw, double* width, double* length,
                  double min_car_width, double min_car_length)
{
  vector<point3d_t>::iterator it;
  double rot_min_x = DBL_MAX;
  double rot_min_y = DBL_MAX;
  double rot_max_x = -DBL_MAX;
  double rot_max_y = -DBL_MAX;

  assert(points->size());

  *yaw = TiEV::normalize_theta(angle);

  // expand the bounds to include all points
  dgc_transform_t t;
  dgc_transform_identity(t);
  dgc_transform_rotate_z(t, -angle);

  double rx, ry;
  double rz = 0.0;
  for (it = points->begin(); it != points->end(); it++) {
    point3d_t& point = *it;
    rx = point.x;
    ry = point.y;
    rz = 0.0;
    dgc_transform_point(&rx, &ry, &rz, t);
    if (rx < rot_min_x) rot_min_x = rx;
    if (rx > rot_max_x) rot_max_x = rx;
    if (ry < rot_min_y) rot_min_y = ry;
    if (ry > rot_max_y) rot_max_y = ry;
  }

  // fit car-sized bounding box
  if (rot_max_y < 0) {
    rot_min_y = min(rot_min_y, min_car_width);
  } else if (rot_min_y > 0) {
    rot_max_y = max(rot_max_y, rot_min_y + min_car_width);
  }

  if (rot_max_x < 0) {
    rot_min_x = min(rot_min_x, rot_max_x - min_car_length);
  } else if (rot_min_x > 0) {
    rot_max_x = max(rot_max_x, rot_min_x + min_car_length);
  }

  // find center of bounding box
  double rot_x = (rot_min_x + rot_max_x ) / 2;//the abstacle box=> rot_min_x, rot_max_x, rot_min_y, rot_max_y
  double rot_y = (rot_min_y + rot_max_y ) / 2;
  double rot_z = 0.0;

  // extent
  *width = max(rot_max_y - rot_min_y, min_car_width);
  *length = max(rot_max_x - rot_min_x, min_car_length);

  // un-rotate
  dgc_transform_identity(t);
  dgc_transform_rotate_z(t, angle);

  *x = rot_x;
  *y = rot_y;
  dgc_transform_point(x, y, &rot_z, t);
//  cout<<"this obstacle center_x: "<<*x<<" center_y:"<<*y<<endl;
}


// fit a car-shaped bounding box to an obstacle, given the angle with which to align it

void bounding_box(Obstacle* obstacle, double angle, double min_car_width, double min_car_length)
{
  bounding_box(&obstacle->getPoints(), angle, &obstacle->pose.x, &obstacle->pose.y, &obstacle->pose.yaw, &obstacle->width, &obstacle->length, min_car_width, min_car_length);
}

// will find the principal axis of a point cloud
// works best for rectilinear objects
float align_points(vector<point3d_t>& points) {
  static const int num_bins = 30;
  static const double radians_per_bin = (M_PI_2 / num_bins);
  static const int max_samples = 200;
  static unsigned int count[num_bins];
  static double angle_total[num_bins];

  for (int i=0; i < num_bins; i++) {
    count[i] = 0;
    angle_total[i] = 0;
  }

  int num_points = points.size();
  int samples = min(2*num_points, max_samples);
  int num_samples = 0;
  unsigned int max_count = 0;
  int max_index = 0;
  while (num_samples < max_samples) {
    point3d_t p1 = points[rand() % num_points];
    point3d_t p2 = points[rand() % num_points];
    double dy = (p1.y - p2.y);
    double dx = (p1.x - p2.x);
    if ((fabs(dy) < 0.01) && ( fabs(dx) < 0.01))
      continue;
    double y = atan2(dy, dx);

    // wrap into range
    if (y < 0) y += TiEV_PI;
    if (y >= M_PI_2) y -= M_PI_2;

    int idx = (num_bins * y / M_PI_2);
    if (idx >= num_bins) {
      idx = 0;
      y = 0.0;
    }
    angle_total[idx] += y;
    count[idx]++;
    if (count[idx] > max_count) {
      max_count = count[idx];
      max_index = idx;
    }

    num_samples++;
  }

  return angle_total[max_index] / max_count;
 }

float align_obstacle(Obstacle* obstacle) {
  return align_points(obstacle->getPoints());
}

// determine the alignment (fit a car-shaped bounding box to an obstacle with PCA alignment
void align_bounding_box(Obstacle* obstacle, double min_car_width, double min_car_length)
{
  vector<point3d_t>& points = obstacle->getPoints();
  double yaw;

  vector<point3d_t>::iterator it;
  double mean_x = 0, mean_y = 0, mean_z = 0;
  float xc, yc;
  float a = 0 , b = 0 , d = 0;

  int count = points.size();
//  cout<<"this obstacle has "<<count<<" points"<<endl;
  assert(count);


  for (it = points.begin(); it != points.end(); it++)
  {
    
    mean_x += it->x;
    mean_y += it->y;
    mean_z += it->z;
  }

  mean_x /= count;
  mean_y /= count;
  mean_z /= count;

  for (it = points.begin(); it != points.end(); it++)
  {
    xc = it->x - mean_x;
    yc = it->y - mean_y;

    a += xc*xc;
    b += xc*yc;
    d += yc*yc;
  }

  a /= count-1;//varX
  b /= count-1;//covarXY
  d /= count-1;//varY

  float D=a*d-b*b;
  float T=a+d;

  float tmp=sqrt(T*T/4-D);

  float L1=T/2.+ tmp;
//  float L2=T/2.- tmp;

  yaw=-atan2(L1-d,b);

  bounding_box(obstacle, yaw, min_car_width, min_car_length);
}
