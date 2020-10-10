//
// Created by xlz on 17-9-27.
//
#include "node_option.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

NodeOptions CreateNodeOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
    NodeOptions options;
    options.map_builder_options =
            ::cartographer::mapping::CreateMapBuilderOptions(
                    lua_parameter_dictionary->GetDictionary("map_builder").get());
    options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    options.lookup_transform_timeout_sec =
            lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    options.submap_publish_period_sec =
            lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
    options.pose_publish_period_sec =
            lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
    options.trajectory_publish_period_sec =
            lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
    return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
        const string& configuration_directory,
        const string& configuration_basename) {
    auto file_resolver = cartographer::common::make_unique<
            cartographer::common::ConfigurationFileResolver>(
            std::vector<string>{configuration_directory});
    const string code =
            file_resolver->GetFileContentOrDie(configuration_basename);
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
            code, std::move(file_resolver));

    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                           CreateTrajectoryOptions(&lua_parameter_dictionary));
}