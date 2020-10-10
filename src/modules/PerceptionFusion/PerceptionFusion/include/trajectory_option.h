//
// Created by xlz on 17-9-27.
//

#ifndef TEST_CARTOGRAPHER_TRAJECTORY_OPTION_H
#define TEST_CARTOGRAPHER_TRAJECTORY_OPTION_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"



    struct TrajectoryOptions {
        ::cartographer::mapping::proto::TrajectoryBuilderOptions
                trajectory_builder_options;
        string tracking_frame;
        string published_frame;
        string odom_frame;
        bool provide_odom_frame;
        bool use_odometry;
        int num_laser_scans;
        int num_multi_echo_laser_scans;
        int num_subdivisions_per_laser_scan;
        int num_point_clouds;
    };

    TrajectoryOptions CreateTrajectoryOptions(
            ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

/*
// Try to convert 'msg' into 'options'. Returns false on failure.
    bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                        TrajectoryOptions* options);

// Converts 'trajectory_options' into a ROS message.
    cartographer_ros_msgs::TrajectoryOptions ToRosMessage(
            const TrajectoryOptions& trajectory_options);
*/
   // namespace cartographer_ros

#endif //TEST_CARTOGRAPHER_TRAJECTORY_OPTION_H
