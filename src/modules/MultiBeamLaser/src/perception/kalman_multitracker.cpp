#include "kalman_multitracker.h"
#include "box.h"
#include <utils.h>
#include <perception.h>
#include <global.h>
#include <sstream> 
#include <math.h>
#include "tievlog.h"
using namespace TiEV;
using namespace std;
using namespace std::tr1
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

KalmanMultiTracker::KalmanMultiTracker(
        double correspondence_thresh, double pruning_thresh,
		double measurement_variance, double position_variance,
		double velocity_variance, double heading_variance,
        double heading_velocity_variance, double initial_position_variance,
		double initial_velocity_variance, double initial_heading_variance,
        double initial_heading_velocity_variance) :
        next_id_(0),
        correspondence_thresh_(correspondence_thresh),
        pruning_thresh_(pruning_thresh),    
        current_timestamp_(0),
        prev_timestamp_(0)
{
    measurement_matrix_ = MatrixXd::Identity(2, 5);
    transition_matrix_ = MatrixXd::Identity(5, 5);
    transition_matrix_mu_ = MatrixXd::Identity(5, 5);
    transition_matrix_sigma_ = MatrixXd::Identity(5, 5);
    measurement_covariance_ = MatrixXd::Identity(2, 2) * measurement_variance;
    initial_sigma_ = MatrixXd::Zero(5, 5);
    initial_sigma_(0, 0) = initial_position_variance;
    initial_sigma_(1, 1) = initial_position_variance;
    initial_sigma_(2, 2) = initial_velocity_variance;
    initial_sigma_(3, 3) = initial_heading_variance;
    initial_sigma_(4, 4) = initial_heading_velocity_variance;


    transition_covariance_ = MatrixXd::Zero(5, 5);
    transition_covariance_(0, 0) = position_variance;
    transition_covariance_(1, 1) = position_variance;
    transition_covariance_(2, 2) = velocity_variance;
    transition_covariance_(3, 3) = heading_variance;
    transition_covariance_(4, 4) = heading_velocity_variance;
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
        if(fabs((*it)->Get_pose().x) > 20 || (*it)->Get_pose().y < -20 || (*it)->Get_pose().y > 70 || (*it)->missed_ > 20) {
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
    //current_timestamp_ is initilized as 0
    prev_timestamp_ = current_timestamp_;
    current_timestamp_ = timestamp;//this frame first package timestamp

    //world coordinate frame: UTM and ENU
    point2d_t carUtmTrans(latestNavInfo.utmX, latestNavInfo.utmY);
    double carYaw = latestNavInfo.mHeading;

    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;

    bool is_no_measure = measurements.empty();
    bool is_no_tracks = tracks_.empty();

    if(is_no_measure && is_no_tracks)
        return;

    //TODO Verify this assertion, because detector should have record the timestamp of the input PC and the timestamp is the timestamp of the PC
    LOG(WARNING)<<"measurements[0]->Get_timestamp() == timestamp:"<<measurements[0]->Get_timestamp() == timestamp;

    //Association
    //compute score matrix by global nearest neighboor
    if(!is_no_measure && !is_no_tracks) 
    {
        MatrixXd scores((int)tracks_.size(), (int)measurements.size());
        list<std::tr1::shared_ptr<TrackedObstacle> >::iterator it = tracks_.begin();
        for(int i = 0; i < (int)tracks_.size(); ++i, ++it) 
        {
            std::tr1::shared_ptr<TrackedObstacle> track = *it;//for every old track_

            for(int j = 0; j < (int)measurements.size(); ++j) 
            {
                const std::tr1::shared_ptr<Obstacle>& measurement = measurements[j];

                // TODO Verify this assertion, because all the timestamps are the timestamp of the first packet 
                LOG(WARNING)<<"measurements[0]->Get_timestamp() == measurement->Get_timestamp():"<<(measurements[0]->Get_timestamp() == measurement->Get_timestamp());

                // TODO Verify filter's timestamp_ is updated by the measurment time
                double delta_t = measurement->Get_timestamp() - track->filter->timestamp_;

                // Compute the prediction for the timestamp of this measurement.
                transition_matrix_sigma_(0, 2) = delta_t * cos(track->filter->mu_(3));
                transition_matrix_sigma_(1, 2) = delta_t * sin(track->filter->mu_(3));
                transition_matrix_sigma_(0, 3) = delta_t * track->filter->mu_(2) * sin(track->filter->mu_(3)) * -1;
                transition_matrix_sigma_(1, 3) = delta_t * track->filter->mu_(2) * cos(track->filter->mu_(3));
                transition_matrix_sigma_(3, 4) = delta_t;

                VectorXd mu_bar(2);

                //TODO What is this?
                int augment = 1;
                mu_bar(0) = augment * delta_t * track->filter->mu_(2) * cos(track->filter->mu_(3)) + track->filter->mu_(0);
                mu_bar(1) = augment * delta_t * track->filter->mu_(2) * sin(track->filter->mu_(3)) + track->filter->mu_(1);

                //obstacle pose in global frame
                VectorXd innovation = mu_bar - measurement->Get_global_pose();

                MatrixXd sigma_bar = transition_matrix_sigma_ * track->filter->sigma_ * transition_matrix_sigma_.transpose() + track->filter->transition_covariance_;
                MatrixXd sigma_bar_inv_pos = sigma_bar.block(0, 0, 2, 2).inverse(); // The covariance matrix for the marginal of a Gaussian is just that block of the full covariance matrix.
                double tempscore = exp(-0.5 * (innovation.transpose() * sigma_bar_inv_pos * innovation)[0]) / sqrt(( 2 * M_PI * sigma_bar.block(0, 0, 2, 2)).determinant());
                if(track->Get_type() == measurement->Get_type())
                    scores(i, j) = tempscore;
                else
                    scores(i, j) = 0.0;
            }
        }
        data_association_.assign(scores, direct_assignment, reverse_assignment);
    }

    // trackers
    int tracker_idx = 0;
    for (auto it = tracks_.begin(); it != tracks_.end(); ++it, ++tracker_idx)
    {
        //no measurement or not found
        if (is_no_measure || direct_assignment.find(tracker_idx) == direct_assignment.end()) 
        {
            (*it)->missed_++;

            if((*it)->missed_ >= MISSED_TOL)
                (*it)->is_dynamic_ = false;

            //TODO Verify prev_timestamp_ should be the same of (*it)->filter->timestamp_
            double delta_t = current_timestamp_ - prev_timestamp_;
            LOG(WARNING)<<"prev_timestamp_ == (*it)->filter->timestamp_):" <<(prev_timestamp_ == (*it)->filter->timestamp_);


            transition_matrix_mu_(0, 2) = delta_t * cos((*it)->filter->mu_(3));
            transition_matrix_mu_(1, 2) = delta_t * sin((*it)->filter->mu_(3));
            transition_matrix_mu_(3, 4) = delta_t;
            transition_matrix_sigma_(0, 2) = delta_t * cos((*it)->filter->mu_(3));
            transition_matrix_sigma_(1, 2) = delta_t * sin((*it)->filter->mu_(3));
            transition_matrix_sigma_(0, 3) = delta_t * (*it)->filter->mu_(2) * sin((*it)->filter->mu_(3)) * -1;
            transition_matrix_sigma_(1, 3) = delta_t * (*it)->filter->mu_(2) * cos((*it)->filter->mu_(3));
            transition_matrix_sigma_(3, 4) = delta_t;

            //TODO Verify the timestamp

            (*it)->filter->predict_ekf(transition_matrix_mu_,transition_matrix_sigma_, (*it)->filter->timestamp_ + current_timestamp_ - prev_timestamp_);

            Eigen::Vector2d kalmanFliterPose((*it)->filter->mu_(0), (*it)->filter->mu_(1)); //new world pose prediction by kalman
            // Eigen::Vector3d global_pose(kalmanFliterPose(0), kalmanFliterPose(1), 0); 
            // Eigen::Vector3d local_pose =  (*it)->positionGlobalToLocal(global_pose, carUtmTrans, carYaw - M_PI_2);  
            
            (*it)->Set_global_pose(kalmanFliterPose(0), kalmanFliterPose(1), kalmanFliterPose(3)); 
            (*it)->Set_type((*it)->getTypeConfidence());
            (*it)->velocity_ = abs(filter->mu_[2]);
            (*it)->angular_velocity_ = filter->mu_[4];
            (*it)->missed_++;

            (*it)->Dynamic_obj_classifier();
        }
        else // found matched measurement
        {
            int m = direct_assignment.find(tracker_idx)->second;

            double delta_t = measurements[m]->Get_timestamp() - (*it)->filter->timestamp_;

            (*it)->missed_ = 0;  

            transition_matrix_mu_(0, 2) = delta_t * cos((*it)->filter->mu_(3));
            transition_matrix_mu_(1, 2) = delta_t * sin((*it)->filter->mu_(3));
            transition_matrix_mu_(3, 4) = delta_t;
            transition_matrix_sigma_(0, 2) = delta_t * cos((*it)->filter->mu_(3));
            transition_matrix_sigma_(1, 2) = delta_t * sin((*it)->filter->mu_(3));
            transition_matrix_sigma_(0, 3) = delta_t * (*it)->filter->mu_(2) * sin((*it)->filter->mu_(3)) * -1;
            transition_matrix_sigma_(1, 3) = delta_t * (*it)->filter->mu_(2) * cos((*it)->filter->mu_(3));
            transition_matrix_sigma_(3, 4) = delta_t;

            (*it)->filter->predict_ekf(transition_matrix_mu_,transition_matrix_sigma_, measurements[m]->Get_timestamp());//call linear kalman to update the mu_ and sigma_
            (*it)->filter->update(measurements[m]->Get_global_pose(), measurements[m]->Get_timestamp());

            Eigen::Vector2d kalmanFliterPose((*it)->filter->mu_(0), (*it)->filter->mu_(1)); //new world position update by kalman
            // Eigen::Vector3d global_pose(kalmanFliterPose(0), kalmanFliterPose(1), 0);
            // Eigen::Vector3d local_pose = (*it)->positionGlobalToLocal(global_pose, carUtmTrans, carYaw - M_PI_2);  

            (*it)->Set_global_pose(kalmanFliterPose(0), kalmanFliterPose(1), kalmanFliterPose(3)); 
            (*it)->setTypeNum(measurements[m]->Get_type());
            (*it)->Set_type((*it)->getTypeConfidence());
            (*it)->velocity_ = abs(filter->mu_[2]);
            (*it)->angular_velocity_ = filter->mu_[4];

            //maintain the lastest measurement
            (*it)->last_observation_ = measurements[m];
            (*it)->last_observations_.push(measurements[m]);
            (*it)->num_observations_++;
            (*it)->missed_ = 0;

            (*it)->Dynamic_obj_classifier();

            //LOG
            if((*it)->Get_velocity() > 30000) {
                printf("---------------------------------------------\n");
                printf("Position uncertainty: %f\n", (*it)->filter->getPositionUncertainty());
                cout << "-------\n" << (*it)->filter->sigma_ << endl << "--------\n";
                cout << "-------\n" << (*it)->filter->mu_ << endl << "--------\n";
                printf("num_obs: %d\n", (*it)->Get_num_observations());
                printf("pos: %f %f\n", (*it)->Get_pose().x, (*it)->Get_pose().y);
            }
        }
    }

    // new track
    for (size_t i = 0; i < measurements.size(); ++i)
    {
        // no tracks or found
        if (!is_no_tracks && reverse_assignment.find(i) != reverse_assignment.end()) 
            continue;
        
        VectorXd initial_state = VectorXd::Zero(5);

        //world position
        initial_state.segment(0, 2) = measurments[i]->Get_global_pose().x; 
        //world yaw
        initial_state(3) = measurements[i]->Get_global_pose()[2];

        //Kalman EKF
        std::tr1::shared_ptr<LinearKalmanFilter> new_kf(new LinearKalmanFilter(next_id_,
            measurements[i]->Get_timestamp(), initial_state, initial_sigma_,
            measurement_matrix_, transition_covariance_,
            measurement_covariance_));

        std::tr1::shared_ptr<TrackedObstacle> new_obs(new TrackedObstacle(next_id_, measurements[i]));
        new_obs->filter = new_kf;
        new_obs->is_dynamic_ = false;

        //maintain the lastest measurement
        new_obs->last_observation_ = measurements[i];
        new_obs->last_observations_.push(measurements[i]);
        new_obs->num_observations_++;

        new_obs->Set_pose(measurements[i]->Get_pose());
        new_obs->velocity_ = abs(filter->mu_[2]);
        new_obs->angular_velocity_ = filter->mu_[4];
        new_obs->missed_ = 0;


        new_obs->setTypeNum(measurements[i]->Get_type());
        new_obs->Set_type(measurements[i]->Get_type());

        tracks_.push_front(new_obs);
        ++next_id_;
    }
}
