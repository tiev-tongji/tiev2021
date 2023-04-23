
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

// #define MinCarDisThres 0.1
#define MinCarObsThres 2 //number of continue observations 
#define MinCarVelThres 0.3 //velocity threshold
// #define MinBicDisThres 0.05
#define MinBicObsThres 2 
#define MinBicVelThres 0.1
// #define MinPedDisThres 0.03
#define MinPedObsThres 2 
#define MinPedVelThres 0.04
#define MinVelYawThres 1.4
#define MinDynaObjConf 2 // number of dynamic belives

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

    // void TrackedObstacle::estimateModel()
    // {
    //     if (Get_type() == OBSTACLE_PEDESTRIAN) {
    //         width = 1.0;
    //         length = 1.0;
    //     } 
    //     else
    //     {
    //         bounding_box(this, pose_.yaw, 0.5, 0.5);
    //     }
    // }

    TrackedObstacle::TrackedObstacle(double timestamp) :
    time_(timestamp){}

    TrackedObstacle::TrackedObstacle(int id, std::tr1::shared_ptr<Obstacle> observation) :
            id_(id),
            Obstacle(*observation),//*observation
            lastObservation_(observation),
            // pedestrian_label_count(0),
            num_observations_(0)
            {
                dynamic_confidence_ = 0;
                missed_ = 0;
                memset(typeArr_, 0, sizeof(typeArr_));
            }

/*
 * Straight copy constructor
 */
    // TrackedObstacle::TrackedObstacle (const TrackedObstacle& o) :
    //         Obstacle(o),
    //         dynamic_confidence_(o.dynamic_confidence_),
    //         velocity_(o.velocity_),
    //         angular_velocity_(o.angular_velocity_),
    //         isDynamic_(o.isDynamic_),
    //         missed_(o.missed_),

    //         lastObservation_(o.lastObservation_),
    //         num_observations_(o.num_observations_),
    //         // timestamp_first_(o.timestamp_first_),
    //         // timestamp_prediction_(o.timestamp_prediction_),
    //         // timestamp_observation_(o.timestamp_observation_),
    //         // log_odds_(o.log_odds_)
    //         {
    //             assert(num_observations_ > 0);
    //             memset(typeArr_, 0, sizeof(typeArr_));
    //        }

    TrackedObstacle::~TrackedObstacle() {

    }
    void TrackedObstacle::setConfidence(int times)
    {
       dynamic_confidence_ += times;
       if (dynamic_confidence_ < 0)
            dynamic_confidence_ = 0;
       if (dynamic_confidence_ > 10)
            dynamic_confidence_ = 10;
    }

    void TrackedObstacle::Dynamic_obj_classifier()
    {
    //    // TODO what is this?
    //    if (trackGlobalTrajectory_.size() > 8)
    //    {
    //         trackGlobalTrajectory_.erase(trackGlobalTrajectory_.begin());
    //    }

    //    trackGlobalTrajectory_.push_back(point2d_t(global_pose_(0), global_pose_(1)));

    //    int trajSize = trackGlobalTrajectory_.size();

    //    double disdelta = 0;

    //    std::vector<point2d_t> trackLocalTrajectory(trajSize);

    //    for (int i = 0; i < trajSize; ++i)
    //    {
    //         Eigen::Vector3f global_pose(trackGlobalTrajectory_[i].x, trackGlobalTrajectory_[i].y, 0), local_pose;
    //         positionGlobalToLocal(global_pose, translation, rotationAngle, local_pose);
    //         trackLocalTrajectory[i] = (point2d_t(local_pose(0), local_pose(1)));

    //         if (i > 0)
    //         {
    //           disdelta += hypot(trackGlobalTrajectory_[i].x - trackGlobalTrajectory_[i - 1].x, trackGlobalTrajectory_[i].y - trackGlobalTrajectory_[i - 1].y);
    //           // Scalar sca = Scalar(255, 255, 0);
    //           // myVisual.drawLine(trackLocalTrajectory[i-1], trackLocalTrajectory[i], sca);
    //         }
    //    }
    //    if (trajSize > 1)
    //         disdelta /= (trajSize - 1);

        int confidenceInc = -1;

        if(lastObservation_->Get_type() == OBSTACLE_CAR) //car
        {
            // if(disdelta > MinCarDisThres && getVelocity() > MinCarVelThres)
                // confidenceInc = 1;
            if(getNumObservations() > MinCarObsThres && getVelocity() > MinCarVelThres)   confidenceInc = 1;
        }

        if(lastObservation_->Get_type() == OBSTACLE_BICYCLIST) //bicyclist
        {
            // if(disdelta > MinBicDisThres && getVelocity() > MinBicVelThres)
                // confidenceInc = 1;
            if(getNumObservations() > MinBicObsThres && getVelocity() > MinBicVelThres)   confidenceInc = 1;
        }

        if(lastObservation_->Get_type() == OBSTACLE_PEDESTRIAN) //people
        {
            // if(disdelta > MinPedDisThres && getVelocity() > MinPedVelThres)
                // confidenceInc = 1;
            if(getNumObservations() > MinPedObsThres && getVelocity() > MinBicVelThres)   confidenceInc = 1;
        }

        setConfidence(confidenceInc);
        isDynamic_ = dynamic_confidence_ > MinDynaObjConf ? true : false;

        // if(isDynamic_)
        // {
        //     // Vector2d diffRelativeObj;
        //     // int from = trajSize - 1, to = max(0, trajSize - 6);
        //     // diffRelativeObj(0) = trackGlobalTrajectory_[from].x - trackGlobalTrajectory_[to].x;
        //     // diffRelativeObj(1) = trackGlobalTrajectory_[from].y - trackGlobalTrajectory_[to].y;

        //     double globalObjYaw;
        //     if(latestNavInfo.mRTKStatus == 1)
        //     {
        //         // if(lastObservation_->Get_type() == 0 && 
        //         //   ( getVelocity() < MinVelYawThres || disdelta < MinCarDisThres))
        //         // {
        //         //     pose_.yaw = lastObservation_->Get_pose().yaw;
        //         //         return;
        //         // }

        //         // if(getVelocity() < MinVelYawThres)
        //         //     globalObjYaw = atan2(getYVel(), getXVel());
        //         // else 
        //         //     globalObjYaw = atan2(diffRelativeObj(1), diffRelativeObj(0));
        //         globalObjYaw = filter->mu_(3);
        //         pose_.yaw = globalObjYaw - rotationAngle - M_PI_2;
        //         if (filter->mu_(2) < 0) pose_.yaw = pose_.yaw + M_PI ;
        //     }
        //     else 
        //     {
        //         pose_.yaw = lastObservation_->Get_pose().yaw;
        //         return;
        //     }

        //     while(pose_.yaw > M_PI)
        //     {
        //         pose_.yaw -= 2 * M_PI;
        //     }
        //     while(pose_.yaw < -M_PI)
        //     {
        //         pose_.yaw += 2 * M_PI;
        //     }
        // }
        // else {
        //     pose_.yaw = lastObservation_->Get_pose().yaw;
        // }
    }

    // void TrackedObstacle::update(double timestamp)
    // {
    //     pose_.x = trackGridXY_(0);
    //     pose_.y = trackGridXY_(1);
    //     velocity_ = abs(filter->mu_[2]);
    //     angular_velocity_ = filter->mu_[4];
    //     missed_++;
    // }

    // void TrackedObstacle::update(std::tr1::shared_ptr<Obstacle> obstacle, double timestamp)
    // {
    //     lastObservation_ = obstacle;
    //     timestamp_observation_ = obstacle->Get_timestamp();
    //     num_observations_++;

    //     pose_.x = trackGridXY_(0);
    //     pose_.y = trackGridXY_(1);
    //     velocity_ = abs(filter->mu_[2]);
    //     angular_velocity_ = filter->mu_[4];
    //     missed_ = 0;
    // }

    double TrackedObstacle::getVelocity() const {
        return velocity_;
    }

    double TrackedObstacle::getAngularVel() const
    {
      return angular_velocity_;
    }
    
    void TrackedObstacle::markDynamic() {

        lastObservation_->Set_pose(pose_.x, pose_.y, pose_.z, pose_.yaw, length_, width_);
        lastObservation_->Set_type(this->Get_type());
        // transform( length_/2.0,  width_/2.0, pose_.yaw, pose_.x, pose_.y, x1, y1);
        // transform( length_/2.0, -width_/2.0, pose_.yaw, pose_.x, pose_.y, x2, y2);
        // transform(-length_/2.0, -width_/2.0, pose_.yaw, pose_.x, pose_.y, x3, y3);
        // transform(-length_/2.0,  width_/2.0, pose_.yaw, pose_.x, pose_.y, x4, y4);

        // // lastObservation_->setBoundbox(point2d_t(x1, y1), point2d_t(x2, y2), point2d_t(x3, y3), point2d_t(x4, y4));

        Scalar sca;
        switch(Get_type())
        {
            case OBSTACLE_CAR:
            {
                sca = Scalar(0,0,255);
                break;
            }
            case OBSTACLE_BICYCLIST:
            {
                sca = Scalar(0,255,0);
                break;
            }
            case OBSTACLE_PEDESTRIAN:
            {
                sca = Scalar(0,255,255);
                break;
            }
            default : 
            {
                sca = Scalar(0,50,50);
                break;
            }
        }
                
        if( isDynamic_ && Get_type() != 127)
        {
            myVisual.drawLine(Get_boundbox().p1, Get_boundbox().p2, sca);
            myVisual.drawLine(Get_boundbox().p2, Get_boundbox().p3, sca);
            myVisual.drawLine(Get_boundbox().p3, Get_boundbox().p4, sca);
            myVisual.drawLine(Get_boundbox().p4, Get_boundbox().p1, sca);

            // fill the trajectory
            // trackLocalTrajectory_.push_back(pose_);
            trackGlobalTrajectory_.push_back(global_pose_);

            double tmp_local_heading = Get_pose().yaw;
            double tmp_global_heading = Get_global_pose().yaw;
            for (int i = 1; i < PREDICT_HORIZON; ++i)
            {
                double xx, yy;

                // local
                transform(i * velocity_, 0, tmp_local_heading, Get_pose().x, Get_pose().y, xx, yy);
                // trackLocalTrajectory_.push_back(point2d_t(xx, yy));
                // draw in local
                // myVisual.drawLine(trackLocalTrajectory_[i - 1], point2d_t(xx, yy), sca);
                myVisual.drawLine(point2d_t(Get_pose().x, Get_pose().y), point2d_t(xx, yy), sca);
                myVisual.drawCircle(point2d_t(xx, yy), sca, 1);

                //global
                transform(i * velocity_, 0, tmp_global_heading, Get_global_pose().x, Get_global_pose().y, xx, yy);
                trackGlobalTrajectory_.push_back(point2d_t(xx, yy));

                //
                tmp_local_heading += getAngularVel();
                tmp_global_heading += getAngularVel();
            }

        }
    }
    // void TrackedObstacle::populatePoints() {
    //     this->points_ = lastObservation_->getPoints();
    // }

    // float TrackedObstacle::maxHeight() {
    //     return 0.0; // TODO:??
    // }

}

