#include <grid.h>
#include <Eigen/Dense>
#include <perception_types.h>
#include "perception.h"
#include "obstacle.h"
#include "kalman_multitracker.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <global.h>
#include <algorithm>
#include <iostream>
/******************************************************************
 * INTEGRATION
 ******************************************************************/
#ifndef INTEGRATION_GLOBALS_H_
#define INTEGRATION_GLOBALS_H_

#ifndef OBSTACLE_TYPES_H_
#define OBSTACLE_TYPES_H_

#endif /* OBSTACLE_TYPES_H_ */
#endif /* INTEGRATION_GLOBALS_H_ */

using namespace std;
using namespace std::tr1;
using namespace TiEV;

KalmanMultiTracker* tracker_ = NULL;

float GetCross(const point2d_t& p1, const point2d_t& p2, const point2d_t& p)  
{  
    return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);  
}

bool IsPointInRect(const point2d_t& p1, const point2d_t& p2, const point2d_t &p3, const point2d_t& p4, const point2d_t& p)  
{  
    return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;  
}  


// TODO change to world frame
void perception_prep_obstacles( dgc_grid_p grid, vector<std::tr1::shared_ptr<TiEV::TrackedObstacle> > vec_tracked_obstacles)
{
    myzcm.myObjectList.timestamp = TiEV::getTimeStamp();
    myzcm.myObjectList.data_source = 1;//this result come from mutibeamlazer 2 from vision
    int num_obstacles = vec_tracked_obstacles.size();

    int count = 0;
    int r1, c1, r2, c2, r3, c3, r4, c4;
    int rmin, rmax, cmin, cmax;
    
    for (int i = 0; i < num_obstacles; i++) 
    {
        if (vec_tracked_obstacles[i]->isDynamic_) 
        {
            if(vec_tracked_obstacles[i]->Get_type() == OBSTACLE_UNKNOWN)
                continue;

            myVisual.getRowCol(vec_tracked_obstacles[i]->lastObservation_->Get_boundbox().p1, r1, c1);
            myVisual.getRowCol(vec_tracked_obstacles[i]->lastObservation_->Get_boundbox().p2, r2, c2);
            myVisual.getRowCol(vec_tracked_obstacles[i]->lastObservation_->Get_boundbox().p3, r3, c3);
            myVisual.getRowCol(vec_tracked_obstacles[i]->lastObservation_->Get_boundbox().p4, r4, c4);
            rmin = std::min(std::min(std::min(r1, r2), r3), r4); 
            rmax = std::max(std::max(std::max(r1, r2), r3), r4); 
            cmin = std::min(std::min(std::min(c1, c2), c3), c4); 
            cmax = std::max(std::max(std::max(c1, c2), c3), c4); 

            for(int r = rmin; r <= rmax; r++)
            {
                for(int c = cmin; c <= cmax; c++)
                {
                    if(myVisual.inGrid(r, c))
                    {
                        //remove pointcloud in bounding box
                        if(IsPointInRect(point2d_t(r1, c1), point2d_t(r2, c2), point2d_t(r3, c3), point2d_t(r4, c4), point2d_t(r, c)))
                        {
                            if(myzcm.myLaserMap.cells[r][c] > 0)
                                myzcm.myLaserMap.cells[r][c] = 2;
                            // if(   myVisual.image1.at<Vec3b>(r, c)[0] == 255 
                            // && myVisual.image1.at<Vec3b>(r, c)[1] == 255 
                            // && myVisual.image1.at<Vec3b>(r, c)[2] == 255)
                            // {
                            //     myVisual.image1.at<Vec3b>(r, c)[0] = 0;
                            //     myVisual.image1.at<Vec3b>(r, c)[1] = 0;
                            //     myVisual.image1.at<Vec3b>(r, c)[2] = 0;
                            // }
                        }
                    }
                }
            }

            //define dynamic obect in global frame
            OBJECT dynamicObj;

            dynamicObj.id = count;
            dynamicObj.obj_type = vec_tracked_obstacles[i]->Get_type();
            dynamicObj.width = vec_tracked_obstacles[i]->lastObservation_->Get_width();  
            dynamicObj.length = vec_tracked_obstacles[i]->lastObservation_->Get_length();
            dynamicObj.theta = vec_tracked_obstacles[i]->lastObservation_->Get_global_pose().yaw;

            if(latestNavInfo.mRTKStatus == 1)
                dynamicObj.v = vec_tracked_obstacles[i]->getVelocity();
            else 
                dynamicObj.v = 0;

            dynamicObj.corners.p1_.x = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p1.x;
            dynamicObj.corners.p1_.y = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p1.y;
            dynamicObj.corners.p2_.x = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p2.x;
            dynamicObj.corners.p2_.y = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p2.y;
            dynamicObj.corners.p3_.x = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p3.x;
            dynamicObj.corners.p3_.y = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p3.y;
            dynamicObj.corners.p4_.x = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p4.x;
            dynamicObj.corners.p4_.y = vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox().p4.y;

            for(int j = 0; j < PREDICT_HORIZON; ++j)
            {
                POSITION predictTraj;
                //LOG
                LOG<<"lastObservation_ == tracted objects: " << vec_tracked_obstacles[i]->lastObservation_->Get_pose() == vec_tracked_obstacles[i] ->Get_pose();

                LOG<<"lastObservation_ == tracted objects: " << vec_tracked_obstacles[i]->lastObservation_->Get_global_pose() == vec_tracked_obstacles[i] ->Get_global_pose();
                
                LOG<<"lastObservation_ == tracted objects: " << vec_tracked_obstacles[i]->lastObservation_->Get_global_boundbox() == vec_tracked_obstacles[i] ->Get_global_boundbox();

                //TODO verifiy UTM
                // predictTraj.x = vec_tracked_obstacles[i]->lastObservation_->predictPose[j].x;
                // predictTraj.y = vec_tracked_obstacles[i]->lastObservation_->predictPose[j].y;
                predictTraj.x = vec_tracked_obstacles[i]->trackGlobalTrajectory_[j].x;
                predictTraj.y = vec_tracked_obstacles[i]->trackGlobalTrajectory_[j].y;
                dynamicObj.path.emplace_back(predictTraj);
            }
		    dynamicObj.pathNum = dynamicObj.path.size();
            myzcm.myObjectList.obj.emplace_back(dynamicObj);
            count++;
        }
    }
    myzcm.myObjectList.count = count;
    myVisual.show_image1();
    myzcm.pub_lasermap();
    myzcm.pub_objectlists();
}

//start data association and kalman tracker
void perception_track_obstacles(vector< std::tr1::shared_ptr<Obstacle> >& detected_obstacles, vector< std::tr1::shared_ptr<TrackedObstacle> >& tracked_obstacles, double timestamp)
{
    static bool init = false;
    if (!init) {
        tracker_ = new KalmanMultiTracker(
                settings.kf_settings.correspondence_threshold,
                settings.kf_settings.pruning_threshold,
                settings.kf_settings.measurement_variance,
                settings.kf_settings.position_variance,
                settings.kf_settings.velocity_variance,
                settings.kf_settings.heading_variance,
                settings.kf_settings.heading_velocity_variance,
                settings.kf_settings.initial_position_variance,
                settings.kf_settings.initial_velocity_variance,
                settings.kf_settings.initial_heading_variance,
                settings.kf_settings.initial_heading_velocity_variance
        );
        init = true;
    }
    tracker_->step(detected_obstacles, timestamp);
    tracked_obstacles.clear();
    list<std::tr1::shared_ptr<TrackedObstacle> >::iterator it;
    for(it = tracker_->tracks_.begin(); it != tracker_->tracks_.end(); it++) {
        tracked_obstacles.push_back(*it);
    }
}

void integrate_sensors( dgc_velodyne_data_p velo)
{
    static bool init = false;
    static double last_time;
    static double time0;
    static double delta_s;
    int i;

    handler.slamControlMutex.lock();
    structSLAMCONTROL slamControlFlag = handler.mySlamControl;
    handler.slamControlMutex.unlock();

    if(slamControlFlag.mapping > 0) //only indoor basement mode, use this parameter
    {
	    cout << "enter into slam control mode " << endl;
        settings.z_obstacle_height = 0.28;
    }
    else  //normal driving mode
    {
        if (!init) {
            init = true;
            //read calibration in cal_filename, write to config
            if (dgc_velodyne_read_calibration(settings.velodyne_cal, velo->config) != 0) {
                fprintf(stderr, "# ERROR: could not read calibration file!\n");
                exit(0);
            }

            //difine z grid to identify cell obstacle
            double z_resolution = grid_stat.z_resolution / CM_TO_METER_FACTOR;
            int z_grid_rows = (int) ceil((grid_stat.mapsize.x * grid_stat.resolution) / grid_stat.z_resolution);
            int z_grid_cols = (int) ceil((grid_stat.mapsize.y * grid_stat.resolution) / grid_stat.z_resolution);

            z_grid =
                dgc_grid_initialize(z_resolution,
                                    z_grid_rows,
                                    z_grid_cols,
                                    sizeof(short),
                                    &default_z_cell);
            dgc_grid_recenter_grid(z_grid, 0, 2025);//0
            last_time = TiEV::getTimeStamp();
        }
    }

    grid_stat.center.x = applanix_current_pose()->smooth_x;
    grid_stat.center.y = applanix_current_pose()->smooth_y;
    double temp_x = 0, temp_y = 20.1;//15.1,40.1
    grid->array_c0 = 0;
    grid->array_r0 = 0;

    counter++;

    //generate laser map
    if (settings.use_velodyne) 
    {
        if (velo->num_scans > 0 &&
            (!settings.clear_sensor_data || time0 - velodyne_ts < settings.max_sensor_delay)) {
            integrate_velodyne(velo, counter);
        }
    }

    //start Multiple object tracking
    if (is_mot_enable) 
    {
        counter++;

        if(slamControlFlag.mapping > 0)
        {
            detected_obstacles.clear();
            tracked_obstacles.clear();
        }
        else
        {
            //start tracking
            perception_track_obstacles(detected_obstacles, tracked_obstacles, velo->scans->timestamp);
            std::cout << "kalman tracks = " << tracked_obstacles.size() << std::endl;

            vector< std::tr1::shared_ptr<TrackedObstacle> >::iterator it;
            for(it = tracked_obstacles.begin(); it != tracked_obstacles.end(); it++) 
            {
                //mark dynamic obstacles
                std::tr1::shared_ptr<TrackedObstacle> track = (*it);
                track->markDynamic();
            }
        }
        //publish dynamic dynamic 
        perception_prep_obstacles(grid, tracked_obstacles);
    }

    dgc_perception_map_cell_p cell;
    for(int i = 0; i < myVisual.image1.rows; i++)
    {
        for(int j = 0; j < myVisual.image1.cols; j++) 
        {
            cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, i, j);
            cell->max = -10000;
            cell->min = 10000;
            cell->obstacle = 0;

            myVisual.image1.at<Vec3b>(i,j)[0]=0;
            myVisual.image1.at<Vec3b>(i,j)[1]=0;
            myVisual.image1.at<Vec3b>(i,j)[2]=0;
        }
    }
    last_time = time0;
}
