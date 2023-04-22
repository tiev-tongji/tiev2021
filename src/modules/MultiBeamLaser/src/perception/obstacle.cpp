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

    Obstacle::Obstacle(double time_stamp) :
            time_(time_stamp)
            pose_(zero_pose),
            length_(0.0),
            width_(0.0),
            type_(OBSTACLE_UNKNOWN),
            global_pose_(Eigen::Vector3d::Zero())
    {

    }

    Obstacle::Obstacle(const Obstacle& o) :
            pose_(o.pose),
            length_(o.length),
            width_(o.width),
            type_(o.type_),
    {


    }

    Obstacle::~Obstacle() {

    }

    // // works for convex objects that don't cross grid boundaries (which can happen if grid is recentered)
    // void fast_fill_boundary(dgc_grid_p grid, int start_r, int start_c, unsigned short fill) {
    //     dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, start_r, start_c);

    // }

    // void fill_boundary(dgc_grid_p grid, int start_r, int start_c, int fill) {

    // }

    // void fill_boundary_unsafe(dgc_grid_p grid, int start_r, int start_c, int fill) {

    // }

    //TODO move to nature
    Eigen::Vector3d Obstacle::positionGlobalToLocal(const Eigen::Vector3d &global_pose, const  point2d_t &translation, double rotationAngle)
    {
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(rotationAngle, ::Eigen::Vector3d::UnitZ());
        Eigen::Vector3d trans(-translation.x, -translation.y, 0);
        return R.inverse() * (global_pose + trans);
    }

    //TODO move to nature
    Eigen::Vector3d Obstacle::positionLocalToGlobal(const Eigen::Vector3d &local_pose, const point2d_t &translation, double rotationAngle)
    {
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(rotationAngle, ::Eigen::Vector3d::UnitZ());
        Eigen::Vector3d trans(translation.x, translation.y, 0);
        return  R * local_pose + trans;
    }

    //TODO move to nature
    point2d_t Obstacle::positionGlobalToLocal(const point2d_t &global_pose, const  point2d_t &translation, double rotationAngle)
    {
        Eigen::Vector3d tmp_global_pose(global_pose.x, global_pose.y, 0.0);
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(rotationAngle, ::Eigen::Vector3d::UnitZ());
        Eigen::Vector3d trans(-translation.x, -translation.y, 0);
        Eigen::Vector3d tmp_local_pose =  R.inverse() * (tmp_global_pose + trans);
        return point2d_t(tmp_local_pose.x, tmp_local_pose.y);
    }

    //TODO move to nature
    point2d_t Obstacle::positionLocalToGlobal(const point2d_t &local_pose, const point2d_t &translation, double rotationAngle)
    {
        Eigen::Vector3d tmp_local_pose(local_pose.x, local_pose.y, 0.0);
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(rotationAngle, ::Eigen::Vector3d::UnitZ());
        Eigen::Vector3d trans(translation.x, translation.y, 0);
        Eigen::Vector3d tmp_global_pose =  R * local_pose + trans;
        return point2d_t(tmp_global_pose.x, tmp_global_pose.y);
    }

   
    void Obstacle::Set_pose(double x, double y, double z, double yaw, double length, double width)
    {
        //local pose
        length_ = length;
        width_ = width;
        pose_.x = x;
        pose_.y = y;
        pose_.yaw = yaw;

        //local bbox
        double x1,y1,x2,y2,x3,y3,x4,y4;

        transform( length_/2.0,  width_/2.0, pose_.yaw, pose_.x, pose_.y, x1, y1);
        transform( length_/2.0, -width_/2.0, pose_.yaw, pose_.x, pose_.y, x2, y2);
        transform(-length_/2.0, -width_/2.0, pose_.yaw, pose_.x, pose_.y, x3, y3);
        transform(-length_/2.0,  width_/2.0, pose_.yaw, pose_.x, pose_.y, x4, y4);

        bbox_.p1(x1, y1);
        bbox_.p2(x2, y2);
        bbox_.p3(x3, y3);
        bbox_.p4(x4, y4);

        //global pose
        Eigen::Vector3d local_pose(pose_.x, pose_.y, 0);
        global_pose_ = positionLocalToGlobal(local_pose, latestNavInfo.utmX, latestNavInfo.utmY, latestNavInfo.mHeading - M_PI_2);
        //global yaw
        global_pose_[2] = yaw + latestNavInfo.mHeading - M_PI_2;

        //global bbox
       global_bbox_.p1 = positionLocalToGlobal(bbox_.p1, latestNavInfo.utmX, latestNavInfo.utmY, latestNavInfo.mHeading - M_PI_2);
       global_bbox_.p2 = positionLocalToGlobal(bbox_.p2, latestNavInfo.utmX, latestNavInfo.utmY, latestNavInfo.mHeading - M_PI_2);
       global_bbox_.p3 = positionLocalToGlobal(bbox_.p3, latestNavInfo.utmX, latestNavInfo.utmY, latestNavInfo.mHeading - M_PI_2);
       global_bbox_.p4 = positionLocalToGlobal(bbox_.p4, latestNavInfo.utmX, latestNavInfo.utmY, latestNavInfo.mHeading - M_PI_2);

    }

    void Obstacle::Set_type(dgc_obstacle_type type)
    {
        type_ = type;
    }
    bbox2d_t Obstacle::Get_boundbox()
    {
        return bbox_;
    }
    bbox2d_t Obstacle::Get_global_boundbox()
    {
        return global_bbox_;
    }
    dgc_pose_t Obstacle::Get_pose()
    {
        return pose_;
    }
    Eigen::Vector3d Obstacle::Get_global_pose()
    {
        return global_pose_;
    }
    dgc_obstacle_type Obstacle::Get_type()
    {
        return type_;
    }
    double Obstacle::Get_timestamp()
    {
        return time_;
    }
    // bool Obstacle::getCenterOfPoints(double *x, double *y)
    // {
    //     // if (points_.size() == 0) {
    //     //     populatePoints();
    //     // }
    //     if(x_center_ != 0.0 || y_center_ != 0.0) {
    //         *x = x_center_;
    //         *y = y_center_;
    //         return true;
    //     }

    //     *x = 0;
    //     *y = 0;
    //     int count = points_.size();
    //     if (count < 1)
    //         return false;

    //     for (int i=0; i<count; i++) {
    //         *x = *x + points_[i].x;
    //         *y = *y + points_[i].y;
    //     }

    //     *x = *x / (double)count;
    //     *y = *y / (double)count;
    //     x_center_ = *x;
    //     y_center_ = *y;
    //     return true;
    // }

    // bool Obstacle::getCenterOfPoints(double *x, double *y, double *z)
    // {
    //     // if (points_.size() == 0) {
    //     //     populatePoints();
    //     // }

    //     if(x_center_ != 0.0 || y_center_ != 0.0 || z_center_ != 0.0) {
    //         *x = x_center_;
    //         *y = y_center_;
    //         *z = z_center_;
    //     }

    //     *x = 0;
    //     *y = 0;
    //     *z = 0;
    //     int count = points_.size();
    //     if (count < 1)
    //         return false;

    //     for (int i=0; i<count; i++) {
    //         *x = *x + points_[i].x;
    //         *y = *y + points_[i].y;
    //         *z = *z + points_[i].z;
    //     }

    //     *x = *x / (double)count;
    //     *y = *y / (double)count;
    //     *z = *z / (double)count;
    //     x_center_ = *x;
    //     y_center_ = *y;
    //     z_center_ = *z;
    //     return true;
    // }

    // void Obstacle::merge(const Obstacle& o) {
    //     pose_.x = (pose_.x + o.pose_.x) / 2.0;
    //     pose_.y = (pose_.y + o.pose_.y) / 2.0;
    //     pose_.z = (pose_.z + o.pose_.z) / 2.0;
    //     x_center_ = 0.0;
    //     y_center_ = 0.0;
    //     z_center_ = 0.0;

    //     if ((points_.size() > 0) && (o.points_.size() > 0)) {
    //         points_.insert(points_.end(), o.points_.begin(), o.points_.end());
    //     } else {
    //         points_.clear();
    //     }
    // }

    // void Obstacle::populatePoints() {
    //     points_.clear();
    //     point3d_t pt;
    //     pt.x = pose_.x;
    //     pt.y = pose_.y;
    //     pt.z = pose_.z;
    //     points_.push_back(pt);
    // }

    // std::vector<point3d_t>& Obstacle::getPoints() {
    //     if (points_.size() == 0) {
    //         populatePoints();
    //     }

    //     return points_;
    // }

//     GridObstacle::GridObstacle(int id, dgc_grid_p grid) :
//             Obstacle(id),
//             grid_(grid) {
//     }

//     GridObstacle::GridObstacle (const GridObstacle& o) :
//             Obstacle(o),
//             grid_(o.grid_),
//             cells_(o.cells_) {
//     }

//     GridObstacle::~GridObstacle() {

//     }

//     int GridObstacle::getSize() {
//         return cells_.size();
//     }

//     void GridObstacle::clear() {
//         cells_.clear();
//     }

//     void GridObstacle::merge(const GridObstacle& o) {
//         Obstacle::merge(o);
//         cells_.insert(cells_.end(), o.cells_.begin(), o.cells_.end());
//     }

//     void GridObstacle::addCell(dgc_perception_map_cell_p cell) {
//         cells_.push_back(cell);
//     }

//     std::vector<dgc_perception_map_cell_p>& GridObstacle::getCells() {
//         return cells_;
//     }

//     void GridObstacle::populatePoints() {
//         points_.clear();

//         point3d_t pt;
//         pt.z = 0;
//         for (unsigned int i=0; i < cells_.size(); i++) {
//             points_in_cell(cells_[i], points_);

// //    cell_to_coord(grid_, cells_[i], &pt.x, &pt.y);
// //    points_.push_back(pt);
//         }
//     }

//     float GridObstacle::maxHeight() {
//         float max_height = -1.0;

//         for (unsigned int i=0; i < cells_.size(); i++) {
//             max_height = std::max(max_height, cell_height(cells_[i]));
//         }
//         return max_height;
//     }
}

