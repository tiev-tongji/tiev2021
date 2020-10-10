//
// Created by xlz on 17-9-27.
//

#ifndef TEST_CARTOGRAPHER_NODE_OPTION_H
#define TEST_CARTOGRAPHER_NODE_OPTION_H
#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "trajectory_option.h"



// Top-level options of Cartographer's ROS integration.
    struct NodeOptions {
        ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
        string map_frame;
        double lookup_transform_timeout_sec;
        double submap_publish_period_sec;
        double pose_publish_period_sec;
        double trajectory_publish_period_sec;
    };

    NodeOptions CreateNodeOptions(
            ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

    std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
            const string& configuration_directory,
            const string& configuration_basename);


#endif //TEST_CARTOGRAPHER_NODE_OPTION_H
