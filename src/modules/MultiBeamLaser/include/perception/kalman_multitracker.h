#ifndef KALMAN_MULTITRACKER_H
#define KALMAN_MULTITRACKER_H

#include <vector>
#include <list>
#include <float.h>
#include "obstacle.h"
#include "tracked_obstacle.h"
#include <linear_kalman_filter.h>
#include "data_association.hpp"

class KalmanMultiTracker {
 private:
  //! The the next id_ value to assign to a LinearKalmanFilter.
  int next_id_;
  //! Minimum value of the filter's prediction Gaussian (unnormalized) to allow when making correspondences.
  double correspondence_thresh_;
  //! Maximum allowable position uncertainty before deleting a filter.
  double pruning_thresh_;
  //! Measurement matrix common to all tracks.
  Eigen::MatrixXd measurement_matrix_;
  //! Transition matrix common to all tracks.  Modified every update step based on delta_time.
  Eigen::MatrixXd transition_matrix_;

  //for ekf
  Eigen::MatrixXd transition_matrix_mu_;
  Eigen::MatrixXd transition_matrix_sigma_;

  //! Control matrix common to al tracks.
  Eigen::MatrixXd control_matrix_;
  //! Initial state covariance common to all tracks.
  Eigen::MatrixXd initial_sigma_;
  //! Transition covariance common to all tracks.
  Eigen::MatrixXd transition_covariance_;
  //! Measurement covariance common to all tracks.
  Eigen::MatrixXd measurement_covariance_;
  double current_timestamp_;
  double prev_timestamp_;

  DataAssociation data_association_;
  //! Prune out filters that have high uncertainty.
  void prune( double timestamp );
  void update(const std::vector< std::tr1::shared_ptr<TiEV::Obstacle> >& measurements, double timestamp);

 public:
  std::list<std::tr1::shared_ptr<TiEV::TrackedObstacle> >tracks_;
  
  //! @param initial_position_variance is the variance for both x and y in the initial sigma for new filters.
  KalmanMultiTracker(double correspondence_thresh, double pruning_thresh,
		     double measurement_variance, double position_variance,
		     double velocity_variance, double initial_position_variance,
		     double initial_velocity_variance);

  KalmanMultiTracker(double correspondence_thresh, double pruning_thresh,
		     double measurement_variance, double position_variance,
		     double velocity_variance, double heading_variance,
         double heading_velocity_variance, double initial_position_variance,
		     double initial_velocity_variance, double initial_heading_variance,
         double initial_heading_velocity_variance);

  void eraseAll();

  //! Computes scores, assigns correspondences, runs prediction and update steps for all filters, spawns new filters, prunes bad ones.
  //! @param timestamp is the time at which the update was made; this can be offset arbitrarily from the measurement timestamps.
  void step(const std::vector<std::tr1::shared_ptr <TiEV::Obstacle> >& measurements, double timestamp);
};

#endif //KALMAN_MULTITRACKER_H
