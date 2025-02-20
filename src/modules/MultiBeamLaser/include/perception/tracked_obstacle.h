
#ifndef TRACKED_OBSTACLE_H_
#define TRACKED_OBSTACLE_H_

#include <roadrunner.h>
#include <grid.h>
#include <queue>
#include <vector>
#include <tr1/memory>

#include "obstacle.h"
#include <linear_kalman_filter.h>

namespace TiEV {

class TrackedObstacle : public Obstacle {
private:
  int confidence_;
  int pedestrian_label_count;

  int num_observations_;

  double timestamp_first_;       // time of earliest observation
  double timestamp_prediction_;  // time of most recent prediction

  double x_velocity_;
  double y_velocity_;
  double angular_velocity;

  Eigen::VectorXf prior_log_odds_; //Prior log odds for each class. The indices here correspond to those in booster->class_map_.  
  Eigen::VectorXf log_odds_; //Log odds for each class, updated via incorporateBoostingResponse.
protected:
  virtual void populatePoints();

public:

  bool isDynamic_;
  int typeArr_[128];
  int missed_;
  double trackTheta_;
  dgc_obstacle_type trackType_;
  
  std::vector<point2d_t> trackUtmTrajectory_;
  Eigen::Vector2d trackUtmXY_;
  Eigen::Vector2d trackGridXY_;

  std::tr1::shared_ptr<LinearKalmanFilter> filter;
  std::tr1::shared_ptr<Obstacle> lastObservation_;
  TrackedObstacle(int id, std::tr1::shared_ptr<Obstacle> observation, double timestamp);
  TrackedObstacle(const TrackedObstacle& o);
  virtual ~TrackedObstacle();
 
  void setTypeNum(dgc_obstacle_type type);
  dgc_obstacle_type getTypeConfidence();

  void dynamicClassify(const point2d_t &translation, double rotationAngle);

  void update(std::tr1::shared_ptr<Obstacle>, double timestamp);
  void update(double timestamp);

  void setConfidence(int times) { 
    confidence_ += times; 
    if(confidence_ < 0) 
        confidence_ = 0;
    if(confidence_ > 10) 
        confidence_ = 10;
  }
  int getConfidence() {return confidence_;} 
  
  int getNumObservations() { return num_observations_; }
  
  double getXVel() const;
  double getYVel() const;

  double getVelocity() const;
  double getAngularVel() const;

  Eigen::VectorXf getLogOdds() { return log_odds_; }
  std::tr1::shared_ptr<Obstacle> getLastObservation() { return lastObservation_; }

  virtual int  getSize() { return 0; }
  double timestamp_observation_; // time of most recent observation

  virtual void markDynamic(dgc_grid_p grid, unsigned short counter);
  void estimateModel();

  float maxHeight();
};

}

#endif /* PERCEPTION_OBSTACLE_H_ */
