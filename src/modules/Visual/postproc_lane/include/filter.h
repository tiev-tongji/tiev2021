#ifndef FILTER_H
#define FILTER_H

#define _FILTER_MAX_OBJNUM_ 10000
#define _FILTER_MAX_OBJNUM_DIGIT_ "5"

#include "common.h"
#include "cluster.h"
#include "Eigen/Dense"

template <class Target> class FilterGroup;

template<class Target>
class Survived {

public:

  Survived(const Target& t, const FilterGroup<Target> & _grp, int _id) : grp(_grp), KF(2*t.dim(), t.dim(), 0, CV_32F) {
    det_count = 1;
    mis_count = 0;
    color = cluster::random_color();
    id = _id;
    //KF
    KF.transitionMatrix = cv::Mat_<float>::eye(2*t.dim(), 2*t.dim());
    cv::setIdentity(cv::Mat(KF.transitionMatrix, cv::Rect(t.dim(), 0, t.dim(), t.dim())));
    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
    KF.statePre  = t.full_state();
    KF.statePost = t.full_state();
    //low pass for min_y
    lowpass = 0.0;
    min_y_state = t.min_y();
    //fixed value for y_bias
    y_bias = t.y_bias();
  }
  Survived(const Survived<Target>& other): grp(other.grp) {*this = other;}
  Survived(Survived<Target>&& other) : grp(other.grp) {*this = other;}
  Survived& operator=(const Survived<Target>& other){
    if(&grp != &other.grp) { PRINT("filter assigned between different groups"); }
    KF = other.KF;
    lowpass = other.lowpass;
    min_y_state = other.min_y_state;
    y_bias = other.y_bias;
    det_count = other.det_count;
    mis_count = other.mis_count; 
    color = other.color;
    id = other.id;
    return *this;
  }

  bool operator==(const Survived<Target>& other){
      return fabs(make_target().get_x_bias() - other.make_target().get_x_bias()) < grp.thres_dist;
  }

  bool operator<(const Survived<Target>& other){
      return make_target().get_x_bias() < other.make_target().get_x_bias();
  }

  bool match(Target& new_detection){
    latest_dist = make_target().distance_to(new_detection);
    return latest_dist < grp.thres_dist;
  }
  void update(const Target& new_det){
    KF.predict();
    KF.correct(new_det.state());
    min_y_state = lowpass * min_y_state + (1.0f - lowpass) * new_det.min_y();
    if(det_count <= grp.thres_det_count){
      det_count++;
    }
    mis_count = 0;
  }
  void miss(){
    KF.predict();
    KF.statePost = KF.statePre;
    if(det_count>0){
      det_count--;
    }
    if(mis_count <= grp.thres_mis_count){
      mis_count++;
    }
  }
  bool good() const {
    return det_count > grp.thres_det_count;
  }
  bool dead() const {
    return mis_count > grp.thres_mis_count;
  }
  
  Target make_target() const {
    cv::Mat state = KF.statePost;
    Eigen::Vector3f param;
    param << state.at<float>(0,0), state.at<float>(1,0), state.at<float>(2,0);
    Target target(param, y_bias, min_y_state);
    return target;
  }
  friend std::ostream& operator<<(std::ostream& os, const Survived<Target>& survived_target) {
    os << survived_target.make_target() << "\tdet:" << survived_target.det_count << "\tmis:" << survived_target.mis_count;
    return os;
  }

private:
  cv::KalmanFilter KF;
  const FilterGroup<Target> & grp;
  int det_count;
  int mis_count;
  float latest_dist;
  //lowpass for min_y
  float lowpass;
  float min_y_state;
  //fixed value for y_bias
  float y_bias;

public:
  cv::Vec3b color;
  int id;
};

template <class Target>
class FilterGroup {
  friend Survived<Target>;
  public:
    FilterGroup(
      float _thres_dist,
      int _thres_det_count = 2,
      int _thres_mis_count = 3
    ): thres_dist(_thres_dist), thres_det_count(_thres_det_count), thres_mis_count(_thres_mis_count) {
      obj_counter = 0;
    }

    void update(const std::vector<Target>& _observations) {
      std::vector<Target> raw_observations = _observations;
      std::vector<Target> observations;
      PRINT(filters.size());
      PRINT(observations.size())
      {// remove duplicated observations
        for(Target& raw_obs : raw_observations){
          // check duplicated
          bool duplicated = false;
          for(Target& obs : observations) {
            if (raw_obs.distance_to(obs) < this->thres_dist) {// matched
              duplicated = true;
              if (raw_obs.min_y() < obs.min_y()){
                obs = raw_obs;
              }
              break;
            }// end matched
          }
          if (!duplicated) {
            observations.push_back(raw_obs);
          }
        }
      }
      for(Target& t : observations) {
        PRINT(t)
      }
      for(int i = filters.size() - 1; i>=0; --i) {
        bool matched = false;
        for(int j = observations.size() - 1; j>=0; --j){
          Target & t = observations[j];
          if(t.get_confidence() < 0.5) {
            observations.erase(observations.begin()+j);
            continue;
          }
          if (filters[i].match(t)){
            filters[i].update(t);
            matched = true;
            observations.erase(observations.begin()+j);
            break;
          }
        }
        if (! matched){
          filters[i].miss();
          if (filters[i].dead()){
            PRINT("to be erased")
            PRINT(*(filters.begin()+i))
            PRINT(i)
            filters.erase(filters.begin()+i);
            continue;
          }
        }
        PRINT(filters[i]);
      }
      PRINT("@@")
      for(int i = filters.size() - 1; i>=0; --i) {
        PRINT(filters[i]);
      }
      PRINT("@@")
      PRINT(observations.size())
      PRINT(filters.size());
      for(Target& t : observations) { // Add new filters for unmatched observation
        PRINT(t)
        filters.push_back(Survived<Target>(t, *this, obj_counter++ % _FILTER_MAX_OBJNUM_));
      } 
      PRINT(filters.size());
      { // Remove duplicated filters
          sort(filters.begin(), filters.end());
          filters.erase( unique( filters.begin(), filters.end() ), filters.end() );
      }
      PRINT(filters.size());
    };
    const std::vector<Survived<Target> >& get_survived_targets(){
      return filters;
    }
    void clear(){
        filters.clear();
    }
  private:
    std::vector<Survived<Target> > filters;
	  float thres_dist, thres_det_count, thres_mis_count, lambda, confidence_weight;
    int obj_counter;
};

#endif
