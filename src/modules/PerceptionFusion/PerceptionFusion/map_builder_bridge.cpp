//
// Created by xlz on 17-9-27.
//
#include "map_builder_bridge.h"
#include "trajectory_option.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"

namespace TiEV{

    constexpr double kTrajectoryLineStripMarkerScale = 0.07;
    constexpr double kConstraintMarkerScale = 0.025;

//    ::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
//        ::std_msgs::ColorRGBA result;
//        result.r = color[0];
//        result.g = color[1];
//        result.b = color[2];
//        result.a = 1.f;
//        return result;
//    }
  // namespace

//MapBuilderBridge::MapBuilderBridge(const NodeOptions& node_options,
//                                   tf2_ros::Buffer* const tf_buffer)
//        : node_options_(node_options),
//          map_builder_(node_options.map_builder_options),
//          tf_buffer_(tf_buffer) {}

MapBuilderBridge::MapBuilderBridge(const NodeOptions& node_options)
        : node_options_(node_options),
          map_builder_(node_options.map_builder_options) {}


void MapBuilderBridge::LoadMap(const std::string& map_filename) {
    //LOG(INFO) << "Loading map '" << map_filename << "'...";
    cartographer::io::ProtoStreamReader stream(map_filename);
    map_builder_.LoadMap(&stream);
    // // generate image first;
    // map_builder_.BuildMapImageFromSubmaps("./map_image.png", "./map_image_info.txt", 0.2);
    // // update submaps from modified image, set save_map_flag in params to true
    // map_builder_.UpdateSubmapsFromMapImage("./result/xiqu_map.png", "./result/xiqu_map_modi.png", "./result/xiqu_mapinfo.txt");
}

int MapBuilderBridge::AddTrajectory(
        const std::unordered_set<string>& expected_sensor_ids,
        const TrajectoryOptions& trajectory_options) {
    const int trajectory_id = map_builder_.AddTrajectoryBuilder(
            expected_sensor_ids, trajectory_options.trajectory_builder_options);
    LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

    // Make sure there is no trajectory with 'trajectory_id' yet.
    CHECK_EQ(msg_bridges_.count(trajectory_id), 0);
    msg_bridges_[trajectory_id] =
            cartographer::common::make_unique<MsgBridge>(
                    trajectory_options.num_subdivisions_per_laser_scan,
                    trajectory_options.tracking_frame,
                    //node_options_.lookup_transform_timeout_sec, tf_buffer_,
                    map_builder_.GetTrajectoryBuilder(trajectory_id));
    auto emplace_result =
            trajectory_options_.emplace(trajectory_id, trajectory_options);
    CHECK(emplace_result.second == true);
    return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
    //LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(msg_bridges_.count(trajectory_id), 1);
    map_builder_.FinishTrajectory(trajectory_id);
    msg_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
    //LOG(INFO) << "Running final trajectory optimization...";
    map_builder_.sparse_pose_graph()->RunFinalOptimization();
}

void MapBuilderBridge::SerializeState(const std::string& filename) {
    cartographer::io::ProtoStreamWriter writer(filename);
    map_builder_.SerializeState(&writer);
    CHECK(writer.Close()) << "Could not write state.";
}

MsgBridge* MapBuilderBridge::msg_bridge(const int trajectory_id) {
    return msg_bridges_.at(trajectory_id).get();
}


cv::Mat MapBuilderBridge::GetImageByPose(double reckonx , double reckony ,double yaw, int rows, int cols, int cen_row, int cen_col, float reso ,int search_num ,float threshold)
{
    return map_builder_.sparse_pose_graph()->GetImage(reckonx, reckony, yaw,  rows,  cols,  cen_row,  cen_col,  reso ,search_num , threshold);
}


/*
bool MapBuilderBridge::HandleSubmapQuery(
        cartographer_ros_msgs::SubmapQuery::Request& request,
        cartographer_ros_msgs::SubmapQuery::Response& response) {
    cartographer::mapping::proto::SubmapQuery::Response response_proto;
    cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                              request.submap_index};
    const std::string error =
            map_builder_.SubmapToProto(submap_id, &response_proto);
    if (!error.empty()) {
        LOG(ERROR) << error;
        return false;
    }

    response.submap_version = response_proto.submap_version();
    CHECK(response_proto.textures_size() > 0)
    << "empty textures given for submap: " << submap_id;

    // TODO(gaschler): Forward all textures, not just the first one.
    const auto& texture_proto = *response_proto.textures().begin();
    response.cells.insert(response.cells.begin(), texture_proto.cells().begin(),
                          texture_proto.cells().end());
    response.width = texture_proto.width();
    response.height = texture_proto.height();
    response.resolution = texture_proto.resolution();
    response.slice_pose = ToGeometryMsgPose(
            cartographer::transform::ToRigid3(texture_proto.slice_pose()));
    return true;
}

cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
    cartographer_ros_msgs::SubmapList submap_list;
    submap_list.header.stamp = ::ros::Time::now();
    submap_list.header.frame_id = node_options_.map_frame;
    const auto all_submap_data =
            map_builder_.sparse_pose_graph()->GetAllSubmapData();
    for (size_t trajectory_id = 0; trajectory_id < all_submap_data.size();
         ++trajectory_id) {
        for (size_t submap_index = 0;
             submap_index < all_submap_data[trajectory_id].size(); ++submap_index) {
            const auto& submap_data = all_submap_data[trajectory_id][submap_index];
            if (submap_data.submap == nullptr) {
                continue;
            }
            cartographer_ros_msgs::SubmapEntry submap_entry;
            submap_entry.trajectory_id = trajectory_id;
            submap_entry.submap_index = submap_index;
            submap_entry.submap_version = submap_data.submap->num_range_data();
            submap_entry.pose = ToGeometryMsgPose(submap_data.pose);
            submap_list.submap.push_back(submap_entry);
        }
    }
    return submap_list;
}
*/


std::unordered_map<int, MapBuilderBridge::TrajectoryState> MapBuilderBridge::GetTrajectoryStates() {
    std::unordered_map<int, TrajectoryState> trajectory_states;
    for (const auto& entry : msg_bridges_) {
        const int trajectory_id = entry.first;
        const MsgBridge& msg_bridge = *entry.second;
//        cout << "trajectory_id : " << trajectory_id << endl;

        const cartographer::mapping::TrajectoryBuilder* const trajectory_builder =
                map_builder_.GetTrajectoryBuilder(trajectory_id);
        const cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate =
                trajectory_builder->pose_estimate();
        if (cartographer::common::ToUniversal(pose_estimate.time) < 0) {
            continue;
        }
        // Make sure there is a trajectory with 'trajectory_id'.
        CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
        trajectory_states[trajectory_id] = {
                pose_estimate,
                map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(trajectory_id),//local_to_map
                nullptr/*msg_bridge.tf_bridge().LookupToTracking(
                        pose_estimate.time,
                        trajectory_options_[trajectory_id].published_frame)*/,//published_to_tracking (0,0,0;0,0,0,1) 故不做处理直接给nullptr
                trajectory_options_[trajectory_id]};
    }
    return trajectory_states;
}

/*
visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
    visualization_msgs::MarkerArray trajectory_node_list;
    const auto all_trajectory_nodes =
            map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
    for (int trajectory_id = 0;
         trajectory_id < static_cast<int>(all_trajectory_nodes.size());
         ++trajectory_id) {
        const auto& single_trajectory_nodes = all_trajectory_nodes[trajectory_id];
        visualization_msgs::Marker marker;
        marker.ns = "Trajectory " + std::to_string(trajectory_id);
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.header.stamp = ::ros::Time::now();
        marker.header.frame_id = node_options_.map_frame;
        marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
        marker.scale.x = kTrajectoryLineStripMarkerScale;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.z = 0.05;
        for (const auto& node : single_trajectory_nodes) {
            if (node.trimmed()) {
                continue;
            }
            const ::geometry_msgs::Point node_point =
                    ToGeometryMsgPoint(node.pose.translation());
            marker.points.push_back(node_point);
            // Work around the 16384 point limit in RViz by splitting the
            // trajectory into multiple markers.
            if (marker.points.size() == 16384) {
                trajectory_node_list.markers.push_back(marker);
                ++marker.id;
                marker.points.clear();
                // Push back the last point, so the two markers appear connected.
                marker.points.push_back(node_point);
            }
        }
        trajectory_node_list.markers.push_back(marker);
    }
    return trajectory_node_list;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
    visualization_msgs::MarkerArray constraint_list;
    int marker_id = 0;
    visualization_msgs::Marker constraint_intra_marker;
    constraint_intra_marker.id = marker_id++;
    constraint_intra_marker.ns = "Intra constraints";
    constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
    constraint_intra_marker.header.stamp = ros::Time::now();
    constraint_intra_marker.header.frame_id = node_options_.map_frame;
    constraint_intra_marker.scale.x = kConstraintMarkerScale;
    constraint_intra_marker.pose.orientation.w = 1.0;

    visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
    residual_intra_marker.id = marker_id++;
    residual_intra_marker.ns = "Intra residuals";
    // This and other markers which are less numerous are set to be slightly
    // above the intra constraints marker in order to ensure that they are
    // visible.
    residual_intra_marker.pose.position.z = 0.1;

    visualization_msgs::Marker constraint_inter_marker = constraint_intra_marker;
    constraint_inter_marker.id = marker_id++;
    constraint_inter_marker.ns = "Inter constraints";
    constraint_inter_marker.pose.position.z = 0.1;

    visualization_msgs::Marker residual_inter_marker = constraint_intra_marker;
    residual_inter_marker.id = marker_id++;
    residual_inter_marker.ns = "Inter residuals";
    residual_inter_marker.pose.position.z = 0.1;

    const auto all_trajectory_nodes =
            map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
    const auto all_submap_data =
            map_builder_.sparse_pose_graph()->GetAllSubmapData();
    const auto constraints = map_builder_.sparse_pose_graph()->constraints();

    for (const auto& constraint : constraints) {
        visualization_msgs::Marker *constraint_marker, *residual_marker;
        std_msgs::ColorRGBA color_constraint, color_residual;
        if (constraint.tag ==
            cartographer::mapping::SparsePoseGraph::Constraint::INTRA_SUBMAP) {
            constraint_marker = &constraint_intra_marker;
            residual_marker = &residual_intra_marker;
            // Color mapping for submaps of various trajectories - add trajectory id
            // to ensure different starting colors. Also add a fixed offset of 25
            // to avoid having identical colors as trajectories.
            color_constraint = ToMessage(
                    cartographer::io::GetColor(constraint.submap_id.submap_index +
                                               constraint.submap_id.trajectory_id + 25));
            color_residual.a = 1.0;
            color_residual.r = 1.0;
        } else {
            constraint_marker = &constraint_inter_marker;
            residual_marker = &residual_inter_marker;
            // Bright yellow
            color_constraint.a = 1.0;
            color_constraint.r = color_constraint.g = 1.0;
            // Bright cyan
            color_residual.a = 1.0;
            color_residual.b = color_residual.g = 1.0;
        }

        for (int i = 0; i < 2; ++i) {
            constraint_marker->colors.push_back(color_constraint);
            residual_marker->colors.push_back(color_residual);
        }

        const auto& submap_data =
                all_submap_data[constraint.submap_id.trajectory_id]
                [constraint.submap_id.submap_index];
        const auto& submap_pose = submap_data.pose;
        const auto& trajectory_node_pose =
                all_trajectory_nodes[constraint.node_id.trajectory_id]
                [constraint.node_id.node_index]
                        .pose;
        const cartographer::transform::Rigid3d constraint_pose =
                submap_pose * constraint.pose.zbar_ij;

        constraint_marker->points.push_back(
                ToGeometryMsgPoint(submap_pose.translation()));
        constraint_marker->points.push_back(
                ToGeometryMsgPoint(constraint_pose.translation()));

        residual_marker->points.push_back(
                ToGeometryMsgPoint(constraint_pose.translation()));
        residual_marker->points.push_back(
                ToGeometryMsgPoint(trajectory_node_pose.translation()));
    }

    constraint_list.markers.push_back(constraint_intra_marker);
    constraint_list.markers.push_back(residual_intra_marker);
    constraint_list.markers.push_back(constraint_inter_marker);
    constraint_list.markers.push_back(residual_inter_marker);
    return constraint_list;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
    return sensor_bridges_.at(trajectory_id).get();
}
*/
}