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
#include "common/coordinate_converter/coordinate_converter.h"

using namespace TiEV;

int main() {
    //120.77306897077611  31.59298400919851
    // Angle lat,lon;
    // lat.setByDegree(31.59298400919851);
    // lon.setByDegree(120.77306897077611);
    // LAT lat_(lat);
    // LON lon_(lon);
    // TiEV::WGS84Coor latLonPinvert(lat_, lon_);//(lat, lon);//convert angle to WGS84Coor
    // TiEV::UTMCoor coordinvert;
    // coordinvert = latLonToUTMXY(latLonPinvert);//convert WGS84Coor to UTMxy
    // cout << setprecision(16)  << "UTMxy : " << coordinvert.x << "," << coordinvert.y <<endl;
    // Eigen::Quaterniond quaternion(0.9681809519108286, 0, 0, 0.2502511625488316 );
    // Eigen::Vector3d euler_angles = quaternion.matrix().eulerAngles ( 0,1,2 ); // ZYX顺序，即roll pitch yaw顺序
    // cout << "heading :" << euler_angles[2] <<endl;
    // double heading = euler_angles[2];
    // double dx = 2.3*cos(heading);
    // double dy = 2.3*sin(heading);
    // cout << setprecision(16) << 288772.8836895769+dx << "," << 3497206.347378659 + dy << endl;
    // return 0;

    ParameterReader pd(std::string("../parameters.txt"));

    int mode = atoi(pd.getData("mode").c_str());
    string configuration_directory =pd.getData("configuration_directory");
    string configuration_basename = pd.getData("configuration_basename");

    if( mode == 1 )
    {
        configuration_directory += "/above_ground_mapping&location";
    }
    if( mode == 2 )
    {
        configuration_directory += "/under_ground_mapping";
    }
    if( mode == 3 )
    {
        configuration_directory += "/under_ground_location";
    }

    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
            LoadOptions(configuration_directory, configuration_basename);

    perception_Node node(node_options,mode);

    //loadmap or not
    bool LoadMapFlag = atoi(pd.getData("LoadMapFlag").c_str());
    if (LoadMapFlag == true)
    {
        string map_filename = pd.getData("map_filename");
        node.LoadMap(map_filename + ".pbstream");
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
            f_pose >> pose_x >>pose_y >> pose_z >>
                   orientation_w >>orientation_x >>orientation_y >>orientation_z >> yaw;

            node.initial_tran[0] = pose_x;
            node.initial_tran[1] = pose_y;
            node.initial_tran[2] = pose_z;
            node.initial_rot[0] = orientation_w;
            node.initial_rot[1] = orientation_x;
            node.initial_rot[2] = orientation_y;
            node.initial_rot[3] = orientation_z;
            node.initial_yaw = yaw;
            f_pose.close();
            //cout<<pose_x<<node.initial_tran[1]<<endl;
            node.bInitialPosSet = true;
            node.bLoadFrmMap = true;
        }
        else
        {
            std::cout<<"Fail to load posefile from: "<< pose_file <<std::endl;
            return 0;
        }
    }

    //建立map_builder 并向trajectory_builder添加数据
    node.StartTrajectoryWithDefaultTopics(trajectory_options);

    node.FinishAllTrajectories();

    //removed for updating the map 20201120 John
    // if(mode != 0)
        // node.RunFinalOptimization();

    //savemap or not
    bool SaveMapFlag = atoi(pd.getData("SaveMapFlag").c_str());
    if (SaveMapFlag == true || mode == 2)
    {
        //pbstream
        string save_path = pd.getData("save_path");
        std::tm *now =  TiEV::gettm(TiEV::getTimeStamp());
        string save_file = save_path + std::to_string(now->tm_year+1900) + "_" + std::to_string(now->tm_mon+1) + "_" +
        std::to_string(now->tm_mday) + "_" + std::to_string(now->tm_hour) + "_" + std::to_string(now->tm_min) +
        ".pbstream";
//        string save_file = save_path + "zz.pbstream";
        LOG(INFO) << "Writing state to '" << save_file << "'...";
        node.SerializeState(save_file);
        //initalpose
        string save_file_pose = save_path + std::to_string(now->tm_year+1900) + "_" + std::to_string(now->tm_mon+1) + "_" +
        std::to_string(now->tm_mday) + "_" + std::to_string(now->tm_hour) + "_" + std::to_string(now->tm_min) +
        ".pose";
        LOG(INFO) << "Writing pose to '" << save_file_pose << "'...";
        std::ofstream f_pose(save_file_pose);
        if (f_pose.is_open())
        {
            /* code */
            double initial_x;
            f_pose << setprecision(16) << node.initial_tran[0] <<" "<<node.initial_tran[1] << " "<<node.initial_tran[2] <<
            " "<<node.initial_rot[0] <<" "<<node.initial_rot[1] <<" "<<node.initial_rot[2] <<" "<<
            node.initial_rot[3]<<" "<< node.initial_yaw << std::endl;
            f_pose.close();
            std::cout<<"Finished writing  posefile"<<std::endl;
        }
        else
        {
            std::cout<<"Fail to open posefile for writing"<<std::endl;
            std::cout<<"Current inital pose (x, y) is :"<<node.initial_tran[0] << ", " <<node.initial_tran[1]<< std::endl;
        }
    }
    cout << "Congradulations! finish here!" << endl;
    return 0;
}
