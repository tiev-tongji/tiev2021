
#ifndef TRACKED_OBSTACLE_H_
#define TRACKED_OBSTACLE_H_

#include <roadrunner.h>
#include <grid.h>
#include <queue>
#include <vector>
#include <tr1/memory>

#include "obstacle.h"
#include <linear_kalman_filter.h>

namespace TiEV
{

#define PREDICT_HORIZON 6 // TODO move to param
#define SLIDING_WINDOW 100 //

  // fixed sized queue from stackoverflow
  template <typename T, int MaxLen, typename Container = std::deque<T>>
  class FixedQueue : public std::queue<T, Container>
  {
  public:
    void push(const T &value)
    {
      if (this->size() == MaxLen)
      {
        this->c.pop_front();
      }
      std::queue<T, Container>::push(value);
    }
  };
  class TrackedObstacle : public Obstacle
  {
  private:
    int id_;

    int dynamic_confidence_;
    // int pedestrian_label_count;

    int num_observations_;

    // double timestamp_first_;      // time of earliest observation
    // double timestamp_prediction_; // time of most recent prediction

    // velocity-based motion model
    double velocity_;
    double angular_velocity_;

    // Eigen::VectorXf prior_log_odds_; // Prior log odds for each class. The indices here correspond to those in booster->class_map_.
    // Eigen::VectorXf log_odds_;       // Log odds for each class, updated via incorporateBoostingResponse.

  public:
    bool is_dynamic_;
    int type_filter_[128];
    int missed_;

    // std::vector<point2d_t> trackLocalTrajectory_;
    std::vector<point2d_t> track_global_trajectory_;
    std::tr1::shared_ptr<LinearKalmanFilter> filter;
    
    //This tracked object's all observations
    FixedQueue<std::tr1::shared_ptr<Obstacle>, SLIDING_WINDOW> last_observations_;
    //This tracked object's latest observation
    std::tr1::shared_ptr<Obstacle> last_observation_;

  public:
    TrackedObstacle(double timestamp);
    TrackedObstacle(int id, std::tr1::shared_ptr<Obstacle> observation);
    // TrackedObstacle(const TrackedObstacle &o);
    virtual ~TrackedObstacle();

    void setTypeNum(dgc_obstacle_type type);
    dgc_obstacle_type getTypeConfidence();

    void Dynamic_obj_classifier();

    // void update(std::tr1::shared_ptr<Obstacle>, double timestamp);
    // void update(double timestamp);
    void Set_id(int id){
      id_ = id;
    }
    int Get_id(){
      return id_;
    }
    void setConfidence(int times);
    int getConfidence() { return dynamic_confidence_; }

    int Get_num_observations() { return num_observations_; }

    double Get_velocity() const;
    double getAngularVel() const;

    // Eigen::VectorXf getLogOdds() { return log_odds_; }
    std::tr1::shared_ptr<Obstacle> getLastObservation() { return last_observation_; }

    // virtual int getSize() { return 0; }

    virtual void markDynamic();
    // void estimateModel();

    // float maxHeight();
  };

}

#endif /* PERCEPTION_OBSTACLE_H_ */
