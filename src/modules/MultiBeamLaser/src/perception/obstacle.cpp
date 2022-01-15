#include <perception.h>
#include "obstacle.h"
#include "utils.h"
#include <grid.h>
#include <iostream>
#include <global.h>

using namespace std;
using namespace Eigen;

//#define FILL_DEBUG
//#define FILL_DEBUG_2

void points_in_cell(dgc_perception_map_cell_p cell, std::vector<point3d_t>& points);

dgc_pose_t zero_pose = {0,0,0,0,0,0};

namespace TiEV {

    Obstacle::Obstacle(int id) :
            x_center_(0),
            y_center_(0),
            z_center_(0),
            rndfDist_(-1.0),
            matched_(false),
            id(id),
            pose(zero_pose),
            length(0.0),
            width(0.0),
            second_type(OBSTACLE_UNKNOWN),
            type_this_frame_(OBSTACLE_UNKNOWN),
            classified_this_frame_(false)
            // response_(VectorXf::Zero(getClassNames().size()))
    {
        robot_pose_when_observed_.x = -1;
        robot_pose_when_observed_.y = -1;
        robot_pose_when_observed_.z = -1;
    }

    Obstacle::Obstacle(const Obstacle& o) :
            x_center_(o.x_center_),
            y_center_(o.y_center_),
            z_center_(o.z_center_),
            rndfDist_(o.rndfDist_),
            matched_(o.matched_),
            id (o.id),
            pose(o.pose),
            length(o.length),
            width(o.width),
            second_type(o.second_type),
            type_this_frame_(o.type_this_frame_),
            classified_this_frame_(o.classified_this_frame_),
            robot_pose_when_observed_(o.robot_pose_when_observed_),
            response_(o.response_)
    // import not to copy points_ here
    {


    }

    Obstacle::~Obstacle() {

    }

    // works for convex objects that don't cross grid boundaries (which can happen if grid is recentered)
    void fast_fill_boundary(dgc_grid_p grid, int start_r, int start_c, unsigned short fill) {
        dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, start_r, start_c);

    }

    void fill_boundary(dgc_grid_p grid, int start_r, int start_c, int fill) {

    }

    void fill_boundary_unsafe(dgc_grid_p grid, int start_r, int start_c, int fill) {

    }
    void Obstacle::positionGlobalToLocal(const Eigen::Vector3f &srcGlobal, const  point2d_t &translation, double rotationAngle, Eigen::Vector3f &targetLocal)
    {
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(rotationAngle, ::Eigen::Vector3f::UnitZ());
        Eigen::Vector3f trans(-translation.x, -translation.y, 0);
        targetLocal = R.inverse() * (srcGlobal + trans);
    }

    void Obstacle::positionLocalToGlobal(const Eigen::Vector3f &srcLocal, const point2d_t &translation, double rotationAngle, Eigen::Vector3f &targetGlobal)
    {
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(rotationAngle, ::Eigen::Vector3f::UnitZ());
        Eigen::Vector3f trans(translation.x, translation.y, 0);
        targetGlobal = R * srcLocal + trans;
    }

    //important, show obstacles 
    void Obstacle::markDynamic(dgc_grid_p grid, unsigned short counter, double velocity,double heading,double heading_velocity, bool isdynamic) 
    {
        double x1,y1,x2,y2,x3,y3,x4,y4,x_end,y_end;
        if(length > width)
            swap(length, width);

        // length += 2;
        // width += 1;

        double rotationTheta = pose.yaw - M_PI / 2.0;

        transform( width/2.0,  length/2.0, rotationTheta, pose.x, pose.y, x1, y1);
        transform( width/2.0, -length/2.0, rotationTheta, pose.x, pose.y, x2, y2);
        transform(-width/2.0, -length/2.0, rotationTheta, pose.x, pose.y, x3, y3);
        transform(-width/2.0,  length/2.0, rotationTheta, pose.x, pose.y, x4, y4);
        transform(0,  velocity, rotationTheta, pose.x, pose.y, x_end, y_end);

        setBoundbox(point2d_t(x1, y1), point2d_t(x2, y2), point2d_t(x3, y3), point2d_t(x4, y4));

        Scalar sca;
        switch(second_type)
        {
            case 0:
            {
                sca = Scalar(0,0,255);
                break;
            }
            case 1:
            {
                sca = Scalar(0,255,0);
                break;
            }
            case 2:
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
                
        if( isdynamic && second_type != 127)//isdynamic &&
        // if(second_type != 127)
        {
            // char bufferprint[50];
            // sprintf(bufferprint, "%.1f", pose.yaw * 57.2957795130823208767981548141); //deltadis
            // sprintf(bufferprint, "%.3f", velocity); //deltadis
            // myVisual.drawText(point2d_t(pose.x, pose.y), bufferprint);
            myVisual.drawLine(point2d_t(x1, y1), point2d_t(x2, y2), sca);
            myVisual.drawLine(point2d_t(x2, y2), point2d_t(x3, y3), sca);
            myVisual.drawLine(point2d_t(x3, y3), point2d_t(x4, y4), sca);
            myVisual.drawLine(point2d_t(x4, y4), point2d_t(x1, y1), sca);
            // myVisual.drawLine(point2d_t(pose.x, pose.y), point2d_t(x_end, y_end), sca);

            predictPose[0].x = pose.x;
            predictPose[0].y = pose.y;
            

            for(int i = 1; i < 6; ++i)
            {
                double xx, yy;
                transform(0,  i * velocity, heading, pose.x, pose.y, xx, yy);
                predictPose[i].x = xx;
                predictPose[i].y = yy;
                myVisual.drawLine(point2d_t(predictPose[i-1].x,predictPose[i-1].y), point2d_t(xx, yy), sca);
                myVisual.drawCircle(point2d_t(xx, yy), sca, 1);
                heading = heading + heading_velocity;
            }
        }
    }

    void Obstacle::setBoundbox(point2d_t p1_, point2d_t p2_, point2d_t p3_, point2d_t p4_)
    {
	    p1.x = p1_.x;
	    p1.y = p1_.y;
	    p2.x = p2_.x;
	    p2.y = p2_.y;
	    p3.x = p3_.x;
	    p3.y = p3_.y;
	    p4.x = p4_.x;
	    p4.y = p4_.y;
    }
    
    void Obstacle::setPose(double x_, double y_, double z_, double yaw_, double length_, double width_)
    {
        x_center_ = x_;
        y_center_ = y_;
        z_center_ = z_;
        length = length_;
        width = width_;
        pose.x = x_;
        pose.y = y_;
        pose.yaw = yaw_;
    }


    bool Obstacle::getCenterOfPoints(double *x, double *y)
    {
        // if (points_.size() == 0) {
        //     populatePoints();
        // }
        if(x_center_ != 0.0 || y_center_ != 0.0) {
            *x = x_center_;
            *y = y_center_;
            return true;
        }

        *x = 0;
        *y = 0;
        int count = points_.size();
        if (count < 1)
            return false;

        for (int i=0; i<count; i++) {
            *x = *x + points_[i].x;
            *y = *y + points_[i].y;
        }

        *x = *x / (double)count;
        *y = *y / (double)count;
        x_center_ = *x;
        y_center_ = *y;
        return true;
    }

    bool Obstacle::getCenterOfPoints(double *x, double *y, double *z)
    {
        // if (points_.size() == 0) {
        //     populatePoints();
        // }

        if(x_center_ != 0.0 || y_center_ != 0.0 || z_center_ != 0.0) {
            *x = x_center_;
            *y = y_center_;
            *z = z_center_;
        }

        *x = 0;
        *y = 0;
        *z = 0;
        int count = points_.size();
        if (count < 1)
            return false;

        for (int i=0; i<count; i++) {
            *x = *x + points_[i].x;
            *y = *y + points_[i].y;
            *z = *z + points_[i].z;
        }

        *x = *x / (double)count;
        *y = *y / (double)count;
        *z = *z / (double)count;
        x_center_ = *x;
        y_center_ = *y;
        z_center_ = *z;
        return true;
    }

    void Obstacle::merge(const Obstacle& o) {
        pose.x = (pose.x + o.pose.x) / 2.0;
        pose.y = (pose.y + o.pose.y) / 2.0;
        pose.z = (pose.z + o.pose.z) / 2.0;
        x_center_ = 0.0;
        y_center_ = 0.0;
        z_center_ = 0.0;

        if ((points_.size() > 0) && (o.points_.size() > 0)) {
            points_.insert(points_.end(), o.points_.begin(), o.points_.end());
        } else {
            points_.clear();
        }
    }

    void Obstacle::populatePoints() {
        points_.clear();
        point3d_t pt;
        pt.x = pose.x;
        pt.y = pose.y;
        pt.z = pose.z;
        points_.push_back(pt);
    }

    std::vector<point3d_t>& Obstacle::getPoints() {
        if (points_.size() == 0) {
            populatePoints();
        }

        return points_;
    }

    GridObstacle::GridObstacle(int id, dgc_grid_p grid) :
            Obstacle(id),
            grid_(grid) {
    }

    GridObstacle::GridObstacle (const GridObstacle& o) :
            Obstacle(o),
            grid_(o.grid_),
            cells_(o.cells_) {
    }

    GridObstacle::~GridObstacle() {

    }

    int GridObstacle::getSize() {
        return cells_.size();
    }

    void GridObstacle::clear() {
        cells_.clear();
    }

    void GridObstacle::merge(const GridObstacle& o) {
        Obstacle::merge(o);
        cells_.insert(cells_.end(), o.cells_.begin(), o.cells_.end());
    }

    void GridObstacle::addCell(dgc_perception_map_cell_p cell) {
        cells_.push_back(cell);
    }

    std::vector<dgc_perception_map_cell_p>& GridObstacle::getCells() {
        return cells_;
    }

    void GridObstacle::populatePoints() {
        points_.clear();

        point3d_t pt;
        pt.z = 0;
        for (unsigned int i=0; i < cells_.size(); i++) {
            points_in_cell(cells_[i], points_);

//    cell_to_coord(grid_, cells_[i], &pt.x, &pt.y);
//    points_.push_back(pt);
        }
    }

    float GridObstacle::maxHeight() {
        float max_height = -1.0;

        for (unsigned int i=0; i < cells_.size(); i++) {
            max_height = std::max(max_height, cell_height(cells_[i]));
        }
        return max_height;
    }
}

