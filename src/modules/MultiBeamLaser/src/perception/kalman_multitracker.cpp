#include "kalman_multitracker.h"
#include "box.h"
#include <utils.h>
#include <perception.h>
#include <global.h>
#include <sstream> 
#include <math.h>
using namespace TiEV;
using namespace std;
using namespace std::tr1;
using namespace Eigen;

#define MAX_CAR_SIZE 6
#define MAX_BICYCLE_SIZE 4
#define MAX_PEDES_SIZE 2

KalmanMultiTracker::KalmanMultiTracker(
        double correspondence_thresh, double pruning_thresh,
        double measurement_variance, double position_variance,
        double velocity_variance, double initial_position_variance,
        double initial_velocity_variance) :
        next_id_(0),
        correspondence_thresh_(correspondence_thresh),
        pruning_thresh_(pruning_thresh),    
        current_timestamp_(0),
        prev_timestamp_(0)
{
    measurement_matrix_ = MatrixXd::Identity(2, 4);
    transition_matrix_ = MatrixXd::Identity(4, 4);
    measurement_covariance_ = MatrixXd::Identity(2, 2) * measurement_variance;
    initial_sigma_ = MatrixXd::Zero(4, 4);
    initial_sigma_(0, 0) = initial_position_variance;
    initial_sigma_(1, 1) = initial_position_variance;
    initial_sigma_(2, 2) = initial_velocity_variance;
    initial_sigma_(3, 3) = initial_velocity_variance;

    transition_covariance_ = MatrixXd::Zero(4, 4);
    transition_covariance_(0, 0) = position_variance;
    transition_covariance_(1, 1) = position_variance;
    transition_covariance_(2, 2) = velocity_variance;
    transition_covariance_(3, 3) = velocity_variance;
}

void KalmanMultiTracker::eraseAll()
{
    list<std::tr1::shared_ptr<TrackedObstacle> >::iterator it;
    for(it = tracks_.begin(); it != tracks_.end(); ++it) 
    {
        it = tracks_.erase(it);
        it--;
        continue;
    }
}


void KalmanMultiTracker::prune(double timestamp_current) 
{
    list<std::tr1::shared_ptr<TrackedObstacle> >::iterator it;
    for(it = tracks_.begin(); it != tracks_.end(); ++it) 
    {
        // x, y variances are equal, so just prune on x variance.
        if((*it)->filter->sigma_(0, 0) >= pruning_thresh_) {
           it = tracks_.erase(it);
           it--;
           continue;
        }

        //when the tracked obj is far from our car left and right( > 10m) or back( > 20m) or forward( > 70m), delete it
        if(fabs((*it)->pose.x) > 20 || (*it)->pose.y < -20 || (*it)->pose.y > 70 || (*it)->missed_ > 20) {
            it = tracks_.erase(it);
            it--;
            continue;
        }
    }
}

void KalmanMultiTracker::step(const vector< std::tr1::shared_ptr<Obstacle> >& measurements, double timestamp) 
{
    update(measurements, timestamp);
    prune(timestamp);
}

void KalmanMultiTracker::update(const vector< std::tr1::shared_ptr<Obstacle> >& measurements, double timestamp) 
{
    prev_timestamp_ = current_timestamp_;
    current_timestamp_ = timestamp;//this frame first package timestamp

    point2d_t carUtmTrans(latestNavInfo.utmX, latestNavInfo.utmY);
    double carYaw = latestNavInfo.mHeading;

    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;

    bool isNoMeas = false, isNotracks = false;
    if(measurements.empty())
        isNoMeas = true;
    if(tracks_.empty())
        isNotracks = true;

    if(isNoMeas && isNotracks)
        return;

    //compute score matrix by global nearest neighboor
    if(!isNoMeas && !isNotracks) 
    {
        MatrixXd scores((int)tracks_.size(), (int)measurements.size());
        list<std::tr1::shared_ptr<TrackedObstacle> >::iterator it = tracks_.begin();
        for(int i = 0; i < (int)tracks_.size(); ++i, ++it) 
        {
            std::tr1::shared_ptr<TrackedObstacle> track = *it;//for every old track_

            for(int j = 0; j < (int)measurements.size(); ++j) 
            {
                const std::tr1::shared_ptr<Obstacle>& measurement = measurements[j];

                double delta_t = measurement->time_ - track->filter->timestamp_;

                // Compute the prediction for the timestamp of this measurement.
                transition_matrix_(0, 2) = delta_t;
                transition_matrix_(1, 3) = delta_t;

                VectorXd mu_bar(2);

                double vel = track->getVelocity();
                int augment = 1;

                mu_bar(0) = augment * delta_t * track->filter->mu_(2) + track->filter->mu_(0);
                mu_bar(1) = augment * delta_t * track->filter->mu_(3) + track->filter->mu_(1); 

                double x, y;
                measurement->getCenterOfPoints(&x, &y); 
                Eigen::Vector3f srcLocal(x, y, 0), targetGlobal;
                measurement->positionLocalToGlobal(srcLocal, carUtmTrans, carYaw - M_PI_2, targetGlobal);

                measurement->objWorldPose << targetGlobal(0), targetGlobal(1); //measurement world pose

                VectorXd innovation = mu_bar - measurement->objWorldPose;

                MatrixXd sigma_bar = transition_matrix_ * track->filter->sigma_ * transition_matrix_.transpose() + track->filter->transition_covariance_;
                MatrixXd sigma_bar_inv_pos = sigma_bar.block(0, 0, 2, 2).inverse(); // The covariance matrix for the marginal of a Gaussian is just that block of the full covariance matrix.
                double tempscore = exp(-0.5 * (innovation.transpose() * sigma_bar_inv_pos * innovation)[0]) / sqrt(( 2 * M_PI * sigma_bar.block(0, 0, 2, 2)).determinant());
                if(track->trackType_ == measurement->second_type)
                    scores(i, j) = tempscore;
                else
                    scores(i, j) = 0.0;
            }
        }
        data_association_.assign(scores, direct_assignment, reverse_assignment);
    }

    int tracker_idx = 0;
    for (auto it = tracks_.begin(); it != tracks_.end(); ++it, ++tracker_idx)
    {
        //no measurement or not found
        if (isNoMeas || direct_assignment.find(tracker_idx) == direct_assignment.end()) 
        {
            (*it)->missed_++;

            if((*it)->missed_ >= 10)
                (*it)->isDynamic_ = false;

            transition_matrix_(0, 2) = current_timestamp_ - prev_timestamp_;
            transition_matrix_(1, 3) = current_timestamp_ - prev_timestamp_;

            (*it)->filter->predict(transition_matrix_, (*it)->filter->timestamp_ + current_timestamp_ - prev_timestamp_);

            Eigen::Vector2d kalmanFliterPose((*it)->filter->mu_(0), (*it)->filter->mu_(1)); //new world pose prediction by kalman
            Eigen::Vector3f srcGlobal(kalmanFliterPose(0), kalmanFliterPose(1), 0), targetLocal;     
            (*it)->positionGlobalToLocal(srcGlobal, carUtmTrans, carYaw - M_PI_2, targetLocal);  

            (*it)->trackGridXY_ << targetLocal(0), targetLocal(1);
            (*it)->trackUtmXY_ = kalmanFliterPose;
            (*it)->trackType_ = (*it)->getTypeConfidence();

            (*it)->update(current_timestamp_);

            (*it)->dynamicClassify(carUtmTrans, carYaw - M_PI_2);

        }
        else // found matched measurement
        {
            int m = direct_assignment.find(tracker_idx)->second;

            double delta_time = measurements[m]->time_ - (*it)->filter->timestamp_;

            (*it)->missed_ = 0;  

            transition_matrix_(0, 2) = delta_time;
            transition_matrix_(1, 3) = delta_time;

            (*it)->filter->predict(transition_matrix_, measurements[m]->time_);//call linear kalman to update the mu_ and sigma_
            (*it)->filter->update(measurements[m]->objWorldPose, measurements[m]->time_);

            Eigen::Vector2d kalmanFliterPose((*it)->filter->mu_(0), (*it)->filter->mu_(1)); //new world position update by kalman
            Eigen::Vector3f srcGlobal(kalmanFliterPose(0), kalmanFliterPose(1), 0), targetLocal;     
            (*it)->positionGlobalToLocal(srcGlobal, carUtmTrans, carYaw - M_PI_2, targetLocal);  

            (*it)->trackGridXY_ << targetLocal(0), targetLocal(1);
            (*it)->trackUtmXY_ = kalmanFliterPose;
            (*it)->setTypeNum(measurements[m]->second_type);
            (*it)->trackType_ = (*it)->getTypeConfidence();

            (*it)->update(measurements[m], measurements[m]->time_);//update the obj 

            (*it)->dynamicClassify(carUtmTrans, carYaw - M_PI_2);

            if((*it)->getVelocity() > 30000) {
                printf("---------------------------------------------\n");
                printf("Position uncertainty: %f\n", (*it)->filter->getPositionUncertainty());
                cout << "-------\n" << (*it)->filter->sigma_ << endl << "--------\n";
                cout << "-------\n" << (*it)->filter->mu_ << endl << "--------\n";
                printf("num_obs: %d\n", (*it)->getNumObservations());
                printf("pos: %f %f\n", (*it)->pose.x, (*it)->pose.y);
            }
        }
    }

    for (size_t i = 0; i < measurements.size(); ++i)
    {
        // no tracks or found
        if (!isNotracks && reverse_assignment.find(i) != reverse_assignment.end()) 
            continue;
        
        VectorXd initial_state = VectorXd::Zero(4);

        double x,y;
        measurements[i]->getCenterOfPoints(&x, &y);
        Eigen::Vector3f srcLocal(x, y, 0), targetGlobal;
        measurements[i]->positionLocalToGlobal(srcLocal, carUtmTrans, carYaw - M_PI_2, targetGlobal);
        measurements[i]->objWorldPose << targetGlobal(0), targetGlobal(1);

        initial_state.segment(0, 2) = measurements[i]->objWorldPose; //initial measurement world pose

        std::tr1::shared_ptr<LinearKalmanFilter> new_kf(new LinearKalmanFilter(next_id_,
            measurements[i]->time_, initial_state, initial_sigma_,
            measurement_matrix_, transition_covariance_,
            measurement_covariance_));

        std::tr1::shared_ptr<TrackedObstacle> new_obs(new TrackedObstacle(next_id_, measurements[i], current_timestamp_));
        new_obs->filter = new_kf;
        new_obs->trackUtmXY_ = measurements[i]->objWorldPose;
        new_obs->isDynamic_ = false;
        new_obs->trackGridXY_ << x , y; //initial track local grid pose

        new_obs->update(measurements[i], measurements[i]->time_);
        new_obs->setTypeNum(measurements[i]->second_type);
        new_obs->trackType_ = measurements[i]->second_type;

        tracks_.push_front(new_obs);
        ++next_id_;
    }
}
