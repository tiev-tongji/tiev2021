#ifndef PERCEPTION_OBSTACLE_H_
#define PERCEPTION_OBSTACLE_H_

#include <roadrunner.h>
#include <grid.h>
#include <vector>
// #include <aw_roadNetwork.h>
#include "perception_types.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace TiEV
{

  inline void transform(double x, double y, double theta, double tx, double ty, double &x1, double &y1)
  {
    x1 = cos(theta) * x - sin(theta) * y + tx;
    y1 = sin(theta) * x + cos(theta) * y + ty;
  }

  class Obstacle
  {

  private:
    double time_; // TODO: take better care of observation timestamps: use the position in the spin.  (currently this is just the spin's timestamp, and this may be off by 0.1sec.)

    // dimension
    double length_;
    double width_;

    // class type
    dgc_obstacle_type type_;

    // states in Lidar frame
    dgc_pose_t pose_;
    bbox2d_t bbox_; // p1:right up; p2 right bottom; p3: left bottom; p4 left up
    // states in global frame
    Eigen::Vector3d global_pose_; // utmx utmy yaw(enu)
    bbox2d_t global_bbox_;        //

  public:
    Obstacle(double time_stamp);
    Obstacle(const Obstacle &o);
    virtual ~Obstacle();

  private:
    Eigen::Vector3d positionLocalToGlobal(const Eigen::Vector3d &local_pose, const point2d_t &translation, double rotationAngle);
    point2d_t positionLocalToGlobal(const point2d_t &local_pose, const point2d_t &translation, double rotationAngle);

  public:
    Eigen::Vector3d positionGlobalToLocal(const Eigen::Vector3d &global_pose, const point2d_t &translation, double rotationAngle);
    point2d_t positionGlobalToLocal(const point2d_t &global_pose, const point2d_t &translation, double rotationAngle);

    // set functions
    void Set_pose(double x, double y, double z, double yaw, double length, double width);
    void Set_type(dgc_obstacle_type type);
    // retrive function
    bbox2d_t Get_boundbox();
    bbox2d_t Get_global_boundbox();
    dgc_pose_t Get_pose();
    Eigen::Vector3d Get_global_pose();
    dgc_obstacle_type Get_type();
    double Get_timestamp();
    double Get_width();
    double Get_length();
  };

  // class GridObstacle : public Obstacle {
  // private:
  //   dgc_grid_p grid_;

  // protected:
  //   virtual void populatePoints();

  // public:
  //   std::vector<dgc_perception_map_cell_p> cells_;

  //   GridObstacle(int id, dgc_grid_p grid);
  //   GridObstacle (const GridObstacle& o);
  //   virtual ~GridObstacle();

  //   void addCell(dgc_perception_map_cell_p);
  //   void clear();
  //   std::vector<dgc_perception_map_cell_p>& getCells();
  //   virtual int  getSize();
  //   void merge(const GridObstacle& o);
  //   virtual float maxHeight();
  // //  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
  // };

  // typedef std::vector<Obstacle*> TObstacleVec;

  // class Obstacles {
  // public:
  //   Obstacles() {};
  //   ~Obstacles() {};
  //   dgc_pose_t robot_pose;  // pose of the robot to calculate global obstacle pose from local map
  //   double timestamp;
  //   std::vector<Obstacle*> obstacles;
  // };

}

#endif /* PERCEPTION_OBSTACLE_H_ */
