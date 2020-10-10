//
//  PerceptionFusion
//
//  Created by Xinglian Zhang on 2017/10. modified by 赵君峤 on 2017/10/20.
//  Copyright © 2017年. All rights reserved.
//

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>

#include "cartographer/mapping/map_builder.h"
#include "trajectory_option.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "node_option.h"
#include "node.h"
#include "ParameterReader.h"
#include "common/nature.h"

using namespace TiEV;

int main() {
    ParameterReader pd(std::string("../parameters.txt"));

    int mode = 1;//above ground, for perception fusion
    string configuration_directory =pd.getData("configuration_directory");
    string configuration_basename = pd.getData("configuration_basename");
    configuration_directory += "/above_ground_mapping&location";

    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
            LoadOptions(configuration_directory, configuration_basename);

    perception_Node node_above_ground(node_options,mode);
    //loadmap or not
    bool LoadMapFlag = true;
    if (LoadMapFlag == true)
    {
        string map_filename = pd.getData("map_filename_above_ground");
        node_above_ground.LoadMap(map_filename + ".pbstream");
        std::cout<<"Load map:"<<map_filename<<endl;
        //initialpose
        string pose_file = map_filename + ".pose";
        cout<<"pose_file :"<<pose_file<<endl;
        std::ifstream f_pose(pose_file);
        if (f_pose.is_open())
        {
            /* code */
            cout<<"inininininininini"<<endl;

            double pose_x ,pose_y,pose_z, orientation_w,orientation_x,orientation_y,orientation_z,yaw;
            f_pose >> pose_x >>pose_y >> pose_z
                   >> orientation_w >>orientation_x >>orientation_y >>orientation_z>> yaw;

            node_above_ground.initial_tran[0] = pose_x;
            node_above_ground.initial_tran[1] = pose_y;
            node_above_ground.initial_tran[2] = pose_z;
            node_above_ground.initial_rot[0] = orientation_w;
            node_above_ground.initial_rot[1] = orientation_x;
            node_above_ground.initial_rot[2] = orientation_y;
            node_above_ground.initial_rot[3] = orientation_z;
            node_above_ground.initial_yaw = yaw;
            f_pose.close();
            //cout<<pose_x<<node.initial_tran[1]<<endl;
            node_above_ground.bInitialPosSet = true;
            node_above_ground.bLoadFrmMap = true;
        }
        else
        {
            std::cout<<"Fail to load posefile from: "<< pose_file <<std::endl;
            return 0;
        }
    }
    //finish node
    node_above_ground.StartTrajectoryWithDefaultTopics(trajectory_options);
    node_above_ground.FinishAllTrajectories();
//    node_above_ground.RunFinalOptimization();

    cout << "mode change ========================================" <<endl;

    mode = 3;//pure localization
    configuration_directory =pd.getData("configuration_directory");
    configuration_basename = pd.getData("configuration_basename");
    configuration_directory += "/under_ground_location";

    std::tie(node_options, trajectory_options) =
            LoadOptions(configuration_directory, configuration_basename);

    perception_Node node_under_ground(node_options,mode);
    //loadmap or not
    if (LoadMapFlag == true)
    {
        string map_filename = pd.getData("map_filename_under_ground");
        node_under_ground.LoadMap(map_filename + ".pbstream");
        std::cout<<"Load map:"<<map_filename<<endl;
        //initialpose
        string pose_file = map_filename + ".pose";
        cout<<"pose_file :"<<pose_file<<endl;
        std::ifstream f_pose(pose_file);
        if (f_pose.is_open())
        {
            /* code */
            cout<<"inininininininini"<<endl;

            double pose_x ,pose_y,pose_z, orientation_w,orientation_x,orientation_y,orientation_z,yaw;
            f_pose >> pose_x >>pose_y >> pose_z
                   >> orientation_w >>orientation_x >>orientation_y >>orientation_z>> yaw;

            node_under_ground.initial_tran[0] = pose_x;
            node_under_ground.initial_tran[1] = pose_y;
            node_under_ground.initial_tran[2] = pose_z;
            node_under_ground.initial_rot[0] = orientation_w;
            node_under_ground.initial_rot[1] = orientation_x;
            node_under_ground.initial_rot[2] = orientation_y;
            node_under_ground.initial_rot[3] = orientation_z;
            node_under_ground.initial_yaw = yaw;
            f_pose.close();
            node_under_ground.bInitialPosSet = true;
            node_under_ground.bLoadFrmMap = true;
        }
        else
        {
            std::cout<<"Fail to load posefile from: "<< pose_file <<std::endl;
            return 0;
        }
    }
    //finish node
    node_under_ground.StartTrajectoryWithDefaultTopics(trajectory_options);
    node_under_ground.FinishAllTrajectories();
//    node_under_ground.RunFinalOptimization();

    cout << "Congratulations! Finish here!" <<endl;
    return 0;
}
