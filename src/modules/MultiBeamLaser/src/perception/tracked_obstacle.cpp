
#include "tracked_obstacle.h"
#include "utils.h"
#include "box.h"
#include <assert.h>
#include <global.h>

using namespace std;
using std::tr1::shared_ptr;
using namespace Eigen;

// #define MinCarDisThres 0.2
// #define MinCarVelThres 1
// #define MinPedDisThres 0.1
// #define MinPedVelThres 0.5
// #define MinVelYawThres 1.4
// #define MinDynaObjConf 4

#define MinCarDisThres 0.1
#define MinCarVelThres 0.3
#define MinBicDisThres 0.05
#define MinBicVelThres 0.1
#define MinPedDisThres 0.03
#define MinPedVelThres 0.04
#define MinVelYawThres 1.4
#define MinDynaObjConf 2

namespace TiEV {
    
    void TrackedObstacle::setTypeNum(dgc_obstacle_type newtype)
    {
       if(newtype == OBSTACLE_UNKNOWN)
       {
           dgc_obstacle_type temptype = getTypeConfidence();
           typeArr_[(int)temptype]++;
       }
       else
       {
           typeArr_[(int)newtype]++;
       }
    }

    dgc_obstacle_type TrackedObstacle::getTypeConfidence()
    {
        dgc_obstacle_type newtype = OBSTACLE_UNKNOWN;
        int maxType = 0;
        int maxIndex = 127;
        for(int index = 0; index < 3; index++ )
        {
           if(typeArr_[index] > maxType)
           { 
              maxType = typeArr_[index];
              maxIndex = index;
           }
        }
       switch(maxIndex)
       {
            case 127:
            newtype = OBSTACLE_UNKNOWN;
            break;
            case 0:
            newtype = OBSTACLE_CAR;
            break;
            case 2:
            newtype = OBSTACLE_PEDESTRIAN;
            break;
            case 1:
            newtype = OBSTACLE_BICYCLIST;
            break;
        }
        return newtype;	
   } 

    void TrackedObstacle::estimateModel()
    {
        if (trackType_ == OBSTACLE_PEDESTRIAN) {
            width = 1.0;
            length = 1.0;
        } 
        else
        {
            bounding_box(this, pose.yaw, 0.5, 0.5);
        }
    }

    TrackedObstacle::TrackedObstacle(int id, std::tr1::shared_ptr<Obstacle> observation, double timestamp) :
            Obstacle(*observation),//*observation
            lastObservation_(observation),
            pedestrian_label_count(0),
            num_observations_(0)
            {
                confidence_ = 0;
                missed_ = 0;
                memset(typeArr_, 0, sizeof(typeArr_));
            }

/*
 * Straight copy constructor
 */
    TrackedObstacle::TrackedObstacle (const TrackedObstacle& o) :
            Obstacle(o),
            lastObservation_(o.lastObservation_),
            num_observations_(o.num_observations_),
            timestamp_first_(o.timestamp_first_),
            timestamp_prediction_(o.timestamp_prediction_),
            timestamp_observation_(o.timestamp_observation_),
            log_odds_(o.log_odds_)
            {
                assert(num_observations_ > 0);
                memset(typeArr_, 0, sizeof(typeArr_));
           }

    TrackedObstacle::~TrackedObstacle() {

    }

    void TrackedObstacle::dynamicClassify(const point2d_t &translation, double rotationAngle)
    {
        if(trackUtmTrajectory_.size() > 8)
        {
            trackUtmTrajectory_.erase(trackUtmTrajectory_.begin());
        }
        trackUtmTrajectory_.push_back(point2d_t(trackUtmXY_(0), trackUtmXY_(1)));

        int trajSize = trackUtmTrajectory_.size();

        double disdelta = 0;

        std::vector<point2d_t> trackLocalTrajectory(trajSize);


        for (int i = 0; i < trajSize; ++i)
        {
            Eigen::Vector3f srcGlobal(trackUtmTrajectory_[i].x, trackUtmTrajectory_[i].y, 0), targetLocal;     
            positionGlobalToLocal(srcGlobal, translation, rotationAngle, targetLocal);       
            trackLocalTrajectory[i] = (point2d_t(targetLocal(0), targetLocal(1)));

            if(i > 0)
            {
                disdelta += hypot(trackUtmTrajectory_[i].x - trackUtmTrajectory_[i-1].x, trackUtmTrajectory_[i].y - trackUtmTrajectory_[i-1].y);
                Scalar sca = Scalar(255, 255, 0);
                // myVisual.drawLine(trackLocalTrajectory[i-1], trackLocalTrajectory[i], sca);
            }
        }
        if(trajSize > 1)
            disdelta /= (trajSize - 1);

        int confidenceInc = -1;

        if(lastObservation_->second_type == 0) //car
        {
            if(disdelta > MinCarDisThres && getVelocity() > MinCarVelThres)
                confidenceInc = 1;
        }

        if(lastObservation_->second_type == 1) //bicyclist
        {
            if(disdelta > MinBicDisThres && getVelocity() > MinBicVelThres)
                confidenceInc = 1;
        }

        if(lastObservation_->second_type == 2) //people
        {
            if(disdelta > MinPedDisThres && getVelocity() > MinPedVelThres)
                confidenceInc = 1;
        }

        setConfidence(confidenceInc);
        isDynamic_ = confidence_ > MinDynaObjConf ? true : false;

        if(isDynamic_)
        {
            // Vector2d diffRelativeObj;
            // int from = trajSize - 1, to = max(0, trajSize - 6);
            // diffRelativeObj(0) = trackUtmTrajectory_[from].x - trackUtmTrajectory_[to].x;
            // diffRelativeObj(1) = trackUtmTrajectory_[from].y - trackUtmTrajectory_[to].y;

            double globalObjYaw;
            if(latestNavInfo.mRTKStatus == 1)
            {
                // if(lastObservation_->second_type == 0 && 
                //   ( getVelocity() < MinVelYawThres || disdelta < MinCarDisThres))
                // {
                //     trackTheta_ = lastObservation_->pose.yaw;
                //         return;
                // }

                // if(getVelocity() < MinVelYawThres)
                //     globalObjYaw = atan2(getYVel(), getXVel());
                // else 
                //     globalObjYaw = atan2(diffRelativeObj(1), diffRelativeObj(0));
                globalObjYaw = filter->mu_(3);
                trackTheta_ = globalObjYaw - rotationAngle - M_PI_2;
                if (filter->mu_(2) < 0) trackTheta_ = trackTheta_ + M_PI ;
            }
            else 
            {
                trackTheta_ = lastObservation_->pose.yaw;
                return;
            }

            while(trackTheta_ > M_PI)
            {
                trackTheta_ -= 2 * M_PI;
            }
            while(trackTheta_ < -M_PI)
            {
                trackTheta_ += 2 * M_PI;
            }
        }
        else {
            trackTheta_ = lastObservation_->pose.yaw;
        }
    }


    void TrackedObstacle::update(double timestamp)
    {
        pose.x = trackGridXY_(0);
        pose.y = trackGridXY_(1);
        x_velocity_ = abs(filter->mu_[2]);
        angular_velocity = filter->mu_[4];
        y_velocity_ = 0;
        missed_++;
    }

    void TrackedObstacle::update(std::tr1::shared_ptr<Obstacle> obstacle, double timestamp)
    {
        lastObservation_ = obstacle;
        timestamp_observation_ = obstacle->time_;
        num_observations_++;

        pose.x = trackGridXY_(0);
        pose.y = trackGridXY_(1);
        x_velocity_ = abs(filter->mu_[2]);
        angular_velocity = filter->mu_[4];
        y_velocity_ = 0;
        missed_ = 0;
    }


    double TrackedObstacle::getXVel() const {
        return x_velocity_;
    }

    double TrackedObstacle::getYVel() const {
        return y_velocity_;
    }

    double TrackedObstacle::getVelocity() const {
        return range(getXVel(), getYVel());
    }

    double TrackedObstacle::getAngularVel() const
    {
      return angular_velocity;
    }
    
    void TrackedObstacle::markDynamic(dgc_grid_p grid, unsigned short counter) {
        lastObservation_->pose.x = pose.x;
        lastObservation_->pose.y = pose.y;

        lastObservation_->width = width;
        lastObservation_->length = length;
        
        lastObservation_->pose.yaw = this->trackTheta_;
        lastObservation_->second_type = this->trackType_;

        lastObservation_->markDynamic(grid, counter, this->getVelocity(), this->trackTheta_, this->angular_velocity, this->isDynamic_);
    }

    void TrackedObstacle::populatePoints() {
        this->points_ = lastObservation_->getPoints();
    }

    float TrackedObstacle::maxHeight() {
        return 0.0; // TODO:??
    }

}

