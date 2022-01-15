

#ifndef PERCEPTION_OBSTACLE_H_
#define PERCEPTION_OBSTACLE_H_

#include <roadrunner.h>
#include <grid.h>
#include <vector>
//#include <aw_roadNetwork.h>
#include "perception_types.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace TiEV {

inline void transform(double x, double y, double theta, double tx, double ty, double &x1, double &y1)
{
    x1 = cos(theta) * x - sin(theta) * y + tx;
    y1 = sin(theta) * x + cos(theta) * y + ty;
}

class Obstacle {
private:
  double x_center_;
  double y_center_;
  double z_center_;

protected:
  std::vector<point3d_t> points_;
  double rndfDist_;
  bool matched_;

virtual void populatePoints();

public:

  int id;
  point2d_t p1,p2,p3,p4; //p1:right up; p2 right bottom; p3: left bottom; p4 left up
  double length;
  double width;
  point2d_t predictPose[10];
  dgc_pose_t pose;
  Eigen::Vector2d objWorldPose;
  
  dgc_obstacle_type second_type;
  dgc_obstacle_type type_this_frame_;
  bool classified_this_frame_;
  double time_;  // TODO: take better care of observation timestamps: use the position in the spin.  (currently this is just the spin's timestamp, and this may be off by 0.1sec.)
//  double timestamp_;            // TODO: need to set this
  dgc_pose_t robot_pose_when_observed_;
  Eigen::VectorXf response_;

  Obstacle(int id);
  Obstacle(const Obstacle& o);
  virtual ~Obstacle();
  std::vector<point3d_t>& getPoints();
  void positionLocalToGlobal(const Eigen::Vector3f &srcLocal, const point2d_t &translation, double rotationAngle, Eigen::Vector3f &targetGlobal);
  void positionGlobalToLocal(const Eigen::Vector3f &srcGlobal, const point2d_t &translation, double rotationAngle, Eigen::Vector3f &targetLocal);
  void setBoundbox(point2d_t p1_, point2d_t p2_, point2d_t p3_, point2d_t p4_);
  void setPose(double x_, double y_, double z_, double yaw_, double length_, double width_);
  void merge(const Obstacle& o);
  bool getCenterOfPoints(double *x, double *y);
  bool getCenterOfPoints(double *x, double *y, double *z);

  virtual int  getSize() = 0;
  virtual void markDynamic(dgc_grid_p grid, unsigned short counter, double velocity, double heading, double heading_velocity, bool isdynamic);

  virtual float maxHeight() = 0;

  bool getMatched() const { return matched_; }
  void setMatched(bool matched_) { this->matched_ = matched_; }
};

class GridObstacle : public Obstacle {
private:
  dgc_grid_p grid_;

  
protected:
  virtual void populatePoints();

public:
  std::vector<dgc_perception_map_cell_p> cells_;
  
  GridObstacle(int id, dgc_grid_p grid);
  GridObstacle (const GridObstacle& o);
  virtual ~GridObstacle();
  
  void addCell(dgc_perception_map_cell_p);
  void clear();
  std::vector<dgc_perception_map_cell_p>& getCells();
  virtual int  getSize();
  void merge(const GridObstacle& o);
  virtual float maxHeight();
//  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
};



typedef std::vector<Obstacle*> TObstacleVec;

class Obstacles {
public:
  Obstacles() {};
  ~Obstacles() {};
  dgc_pose_t robot_pose;  // pose of the robot to calculate global obstacle pose from local map
  double timestamp;
  std::vector<Obstacle*> obstacles;
};

}

#endif /* PERCEPTION_OBSTACLE_H_ */
