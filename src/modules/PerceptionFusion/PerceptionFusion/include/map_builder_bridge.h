//
// Created by xlz on 17-9-27.
//

#ifndef TEST_CARTOGRAPHER_MAP_BUILDER_BRIDGE_H
#define TEST_CARTOGRAPHER_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "trajectory_option.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "node_option.h"
#include "msg.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"


namespace TiEV{
    class MapBuilderBridge {
    public:
        struct TrajectoryState {
            cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate;
            cartographer::transform::Rigid3d local_to_map;
            std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
            TrajectoryOptions trajectory_options;
        };

        //MapBuilderBridge(const NodeOptions& node_options, tf2_ros::Buffer* tf_buffer);
        MapBuilderBridge(const NodeOptions& node_options);

        MapBuilderBridge(const MapBuilderBridge&) = delete;
        MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

        void LoadMap(const std::string& map_filename);
        int AddTrajectory(const std::unordered_set<string>& expected_sensor_ids,
                          const TrajectoryOptions& trajectory_options);
        void FinishTrajectory(int trajectory_id);
        void RunFinalOptimization();
        void SerializeState(const string& filename);

        std::unordered_map<int, TrajectoryState> GetTrajectoryStates();
        std::unordered_map<int, TrajectoryOptions> GetTrajectoryOption(){return trajectory_options_;}
        //  zz      bool HandleSubmapQuery(
//  zz                 cartographer_ros_msgs::SubmapQuery::Request& request,
//  zz                 cartographer_ros_msgs::SubmapQuery::Response& response);
//  zz
//  zz         cartographer_ros_msgs::SubmapList GetSubmapList();
//  zz         visualization_msgs::MarkerArray GetTrajectoryNodeList();
//  zz         visualization_msgs::MarkerArray GetConstraintList();
//
        MsgBridge* msg_bridge(int trajectory_id);
        cv::Mat GetImageByPose(double reckonx , double reckony ,double yaw, int rows, int cols, int cen_row, int cen_col, float reso,int search_num ,float threshold);

    private:
        const NodeOptions node_options_;
        cartographer::mapping::MapBuilder map_builder_;
        //tf2_ros::Buffer* const tf_buffer_;

        // These are keyed with 'trajectory_id'.
        std::unordered_map<int, TrajectoryOptions> trajectory_options_;
        std::unordered_map<int, std::unique_ptr<MsgBridge>> msg_bridges_;
    };


}


#endif //TEST_CARTOGRAPHER_MAP_BUILDER_BRIDGE_H
