//
// Created by xlz on 17-9-23.
//
#include "msg.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cstring>
#include<math.h>
//#include "common/coordinate_converter/WGS84UTM.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/transform/rigid_transform.h"
using ::cartographer::sensor::PointCloudWithIntensities;
using namespace cartographer::transform;

//用在计算odom pose信息中
class pose_temp
{
    public:
        double x=0 , y=0, th=0;
        double last_time;
        double curr_time;
};


    //tf参数设置

    pose_temp pose_temp1;
    const Rigid3d odom_to_baselink = MsgBridge::creat_tf_eigen(0,0,0,0,0,0,1);//dx dy dz  + x y z w
    const Rigid3d velolink_to_baselink = MsgBridge::creat_tf_eigen(0,0,0,0,0,0,1);//dx dy dz  + x y z w

    bool HasEcho(float) { return true; } //判断range echo是否有数据
    float GetFirstEcho(float range) { return range; } //返回第一个值 其实没有 原文件因为有echos

    MsgBridge::MsgBridge(
        const int num_subdivisions_per_laser_scan, const string& tracking_frame,
        cartographer::mapping::TrajectoryBuilder* const trajectory_builder)
        : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
          trajectory_builder_(trajectory_builder) {}

//Time格式转成cartographertime
    ::cartographer::common::Time MsgBridge::chang_to_Carto_Time(const Time &stamp) {
        // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
        // exactly 719162 days before the Unix epoch.
        return ::cartographer::common::FromUniversal(
                (stamp.sec +
                 ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                10000000ll +
                (stamp.nsec + 50) / 100);  // + 50 to get the rounding correct.
    }

//odom转cartographer::sensor::OdometryData
    std::unique_ptr<::cartographer::sensor::OdometryData> MsgBridge::ToOdometryData(const gps_odom &msg) {
        const cartographer::common::Time time = chang_to_Carto_Time(msg.header.stamp);

        return ::cartographer::common::make_unique<
                ::cartographer::sensor::OdometryData>(
                ::cartographer::sensor::OdometryData{
                        time, ToRigid3d(msg.pose) * odom_to_baselink.inverse()});
    }


    void MsgBridge::HandleOdometryMessage(const string& sensor_id, const gps_odom &msg)
    {
        std::unique_ptr<::cartographer::sensor::OdometryData> odometry_data =
                ToOdometryData(msg);
        if (odometry_data != nullptr) {
            MsgBridge::trajectory_builder_->AddOdometerData(sensor_id, odometry_data->time,
                                                 odometry_data->pose);
        }
    }


//处理Laserscan
    void MsgBridge::HandleLaserScanMessage(const string &sensor_id, const vector<Eigen::Vector3f>& msg, const Time& time_stamp) {

        const cartographer::common::Time time = chang_to_Carto_Time(time_stamp);
        double seconds_between_points =0;//0.00009;

        for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
            const size_t start_index =
                    msg.size() * i / num_subdivisions_per_laser_scan_;
            const size_t end_index =
                    msg.size() * (i + 1) / num_subdivisions_per_laser_scan_;
            const cartographer::sensor::PointCloud subdivision(msg.begin() + start_index,
                                                               msg.begin() + end_index);
            const cartographer::common::Time subdivision_time =
                    time + cartographer::common::FromSeconds((start_index + end_index) / 2. *
                                                                   seconds_between_points);
            HandleRangefinder(sensor_id, subdivision_time, "velo_link", subdivision);
        }
    }


::cartographer::sensor::PointCloudWithIntensities MsgBridge::ToPointCloudWithIntensities(const laser_scan &msg)
{
    CHECK_GE(msg.range_min, 0.f);
    CHECK_GE(msg.range_max, msg.range_min);
    if (msg.angle_increment > 0.f) {
        CHECK_GT(msg.angle_max, msg.angle_min);
    } else {
        CHECK_GT(msg.angle_min, msg.angle_max);
    }
    PointCloudWithIntensities point_cloud;
    float angle = msg.angle_min;

    //(size_t i = 0; i < msg.ranges.size(); ++i)
    for (size_t i = 0; i < 803; ++i) {
        const auto& echoes = msg.ranges[i];
        if (HasEcho(echoes)) {
            const float first_echo = GetFirstEcho(echoes);
            if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
                const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                point_cloud.points.push_back(rotation *
                                             (first_echo * Eigen::Vector3f::UnitX()));
            }
        }
        angle += msg.angle_increment;
    }
    return point_cloud;
}

    void MsgBridge::HandleLaserScan(const string &sensor_id,
                         ::cartographer::common::Time start_time,
                         const string &frame_id,
                         const ::cartographer::sensor::PointCloud &points,
                         double seconds_between_points) {
        int num_subdivisions_per_laser_scan_ = 10;//trajectory_options.num_subdivisions_per_laser_scan
        for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
            const size_t start_index =
                    points.size() * i / num_subdivisions_per_laser_scan_;
            const size_t end_index =
                    points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
            const cartographer::sensor::PointCloud subdivision(points.begin() + start_index,
                                                               points.begin() + end_index);
            const cartographer::common::Time subdivision_time =
                    start_time + cartographer::common::FromSeconds((start_index + end_index) / 2. *
                                                                   seconds_between_points);
            HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
        }
    }

//处理PointClouds
void MsgBridge::HandlePointCloudMessage(const string &sensor_id, const vector<Eigen::Vector3f>& msg, const Time& time_stamp) {

    const cartographer::common::Time time = chang_to_Carto_Time(time_stamp);

    cartographer::sensor::PointCloud point_cloud;
    for (const auto& point : msg) {
        point_cloud.emplace_back(point[0], point[1], point[2]);
    }
    HandleRangefinder(sensor_id, time, "velo_link",  point_cloud);

//        int num_subdivisions_per_laser_scan_ = 10;//trajectory_options.num_subdivisions_per_laser_scan
//        double seconds_between_points =0;//0.00009;
//
//        for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
//            const size_t start_index =
//                    msg.size() * i / num_subdivisions_per_laser_scan_;
//            const size_t end_index =
//                    msg.size() * (i + 1) / num_subdivisions_per_laser_scan_;
//            const cartographer::sensor::PointCloud subdivision(msg.begin() + start_index,
//                                                               msg.begin() + end_index);
//            const cartographer::common::Time subdivision_time =
//                    time + cartographer::common::FromSeconds((start_index + end_index) / 2. *
//                                                                   seconds_between_points);
//            HandleRangefinder(sensor_id, subdivision_time, "velo_link", subdivision);
//        }
}

    void MsgBridge::HandleRangefinder(const string &sensor_id,
                           ::cartographer::common::Time time,
                           const string &frame_id,
                           const ::cartographer::sensor::PointCloud &ranges) {
//        std::vector<Eigen::Vector3f> points = cartographer::sensor::TransformPointCloud(ranges, velolink_to_baselink.cast<float>());

        trajectory_builder_->AddRangefinderData(
                sensor_id, time, velolink_to_baselink.translation().cast<float>(),
                ranges);

        // write result to file
//        std::string VINS_RESULT_PATH;
//        VINS_RESULT_PATH = "/home/xlz/code_compare/zcm.txt";
//        ofstream fout(VINS_RESULT_PATH.c_str(), ios::app);  //不支持C++11，要加.c_str()
//        fout.setf(ios::fixed, ios::floatfield);
//        //fout << header.stamp.toSec() * 1e9 << ",";
//        fout.precision(5);
//        fout << "-----  " << points.size() << endl;
//        int i=0;
//        for (const auto& point : points) {
//            if (i % 2000 == 0)
//            {
//                fout << point[0] << " " << point[1] << " " << point[2] <<endl;
//            }
//            i++;
//        }
//        fout << "-----" << endl << endl;
//
//        fout.close();
    }


    void MsgBridge::SplitString(const string &s, vector<string> &v, const string &c) {
        string::size_type pos1, pos2;
        pos2 = s.find(c);
        pos1 = 0;
        while (string::npos != pos2) {
            v.push_back(s.substr(pos1, pos2 - pos1));

            pos1 = pos2 + c.size();
            pos2 = s.find(c, pos1);
        }
        if (pos1 != s.length())
            v.push_back(s.substr(pos1));
    }

    Heometry_msgs_Quaternion createQuaternionMsgFromYaw(double yaw) {
        Heometry_msgs_Quaternion q_temp;
        q_temp = setRPY(0.0, 0.0, yaw);
        return q_temp;

    }

    Heometry_msgs_Quaternion setRPY(double Roll, double Pitch, double Yaw) {
        Heometry_msgs_Quaternion q_temp;
        double halfRoll = 0.5 * Roll;
        double halfPitch = 0.5 * Pitch;
        double halfYaw = 0.5 * Yaw;

        double sinRoll = sin(halfRoll);
        double cosRoll = cos(halfRoll);
        double sinPitch = sin(halfPitch);
        double cosPitch = cos(halfPitch);
        double sinYaw = sin(halfYaw);
        double cosYaw = cos(halfYaw);

        q_temp.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
        q_temp.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
        q_temp.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
        q_temp.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

        //normalise
        double mag2 = q_temp.x * q_temp.x + q_temp.y * q_temp.y + q_temp.z * q_temp.z + q_temp.w * q_temp.w;
        if (mag2 != 0 && (fabs(mag2 - 1) > 0.1)) {
            double mag = sqrt(mag2);
            q_temp.x /= mag;
            q_temp.y /= mag;
            q_temp.z /= mag;
            q_temp.w /= mag;
        }

        return q_temp;
    }


    Rigid3d MsgBridge::ToRigid3d(const Pose &pose) {
        return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                       ToEigen(pose.orientation));
    }

    Eigen::Vector3d MsgBridge::ToEigen(const Geometry_msgs_Vector3 &vector3) {
        return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
    }

    Eigen::Quaterniond MsgBridge::ToEigen(const Heometry_msgs_Quaternion &quaternion) {
        return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                                  quaternion.z);
    }

    Rigid3d MsgBridge::creat_tf_eigen(double dx, double dy, double dz, double x, double y, double z, double w) {
        Eigen::Matrix<double, 3, 1> translation;
        translation << dx, dy, dz;
        return Rigid3<double>(translation, Eigen::Quaterniond(w, x, y, z));
    }
