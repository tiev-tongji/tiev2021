/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping_2d/sparse_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <fstream>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"
#include "glog/logging.h"
#include "common/nature.h"

namespace cartographer {
namespace mapping_2d {
//*****zz 19-10-14*****//
mapping::SubmapId submapID_helper{0,0};//save SubmapId temp;
//*****zz 19-10-14*****//
SparsePoseGraph::SparsePoseGraph(
    const mapping::proto::SparsePoseGraphOptions& options,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options()),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

SparsePoseGraph::~SparsePoseGraph() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(work_queue_ == nullptr);
}

std::vector<mapping::SubmapId> SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_.submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (static_cast<size_t>(trajectory_id) >= submap_data.size() ||
        submap_data[trajectory_id].empty()) {
      optimization_problem_.AddSubmap(
          trajectory_id,
          sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0]));
    }
    CHECK_EQ(submap_data[trajectory_id].size(), 1);
    const mapping::SubmapId submap_id{trajectory_id, 0};
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  CHECK(!submap_data.at(trajectory_id).empty());
  const mapping::SubmapId last_submap_id{
      trajectory_id, submap_data.at(trajectory_id).rbegin()->first};
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of 'insertions_submaps.front()'
    // and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose =
        submap_data.at(trajectory_id).at(last_submap_id.submap_index).pose;
    optimization_problem_.AddSubmap(
        trajectory_id,
        first_submap_pose *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0])
                .inverse() *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[1]));
    return {last_submap_id,
            mapping::SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
  const mapping::SubmapId front_submap_id{trajectory_id,
                                          last_submap_id.submap_index - 1};
  CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

void SparsePoseGraph::AddScan(
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->initial_pose);

  common::MutexLocker locker(&mutex_);
  trajectory_nodes_.Append(
      trajectory_id, mapping::TrajectoryNode{constant_data, optimized_pose});
  ++num_trajectory_nodes_;
  trajectory_connectivity_state_.Add(trajectory_id);

  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (trajectory_id >= submap_data_.num_trajectories() ||
      submap_data_.num_indices(trajectory_id) == 0 ||
      submap_data_
              .at(mapping::SubmapId{
                  trajectory_id, submap_data_.num_indices(trajectory_id) - 1})
              .submap != insertion_submaps.back()) {
    // We grow 'submap_data_' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const mapping::SubmapId submap_id =
        submap_data_.Append(trajectory_id, SubmapData());
    submap_data_.at(submap_id).submap = insertion_submaps.back();

    //*****zz 19-10-14*****////help to record submapID
    submapID_helper.submap_index=submap_id.submap_index;
    submapID_helper.trajectory_id=submap_id.trajectory_id;
    //*****zz 19-10-14*****//
  }

  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }

  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  const bool newly_finished_submap = insertion_submaps.front()->finished();

  //*****zz 19-10-14*****//insert into rtree
  double originx_now,originy_now,reso;

  if(newly_finished_submap == true)
  {
      //finish de shi dang qian ID de qian yi zhang
      mapping::SubmapId submapID_temp{submapID_helper.trajectory_id,submapID_helper.submap_index-1};

      Eigen::Vector2d maxgrid = submap_data_.at(submapID_temp).submap->probability_grid().limits().max();
      double maxx= maxgrid.x();
      double maxy= maxgrid.y();
      double limitx = submap_data_.at(submapID_temp).submap->probability_grid().limits().cell_limits().num_x_cells;
      double limity = submap_data_.at(submapID_temp).submap->probability_grid().limits().cell_limits().num_y_cells;
      double originx,originy;//zz
      originx = submap_data_.at(submapID_temp).submap->local_pose().translation().x();
      originy = submap_data_.at(submapID_temp).submap->local_pose().translation().y();

      //add into R-tree
      reso = submap_data_.at(submapID_temp).submap->probability_grid().limits().resolution();
      box b(point(maxx - reso*limity, maxy - reso*limitx), point(maxx, maxy));

      unsigned i;
      if(submap_data_.num_trajectories()>1)
      {
          i = submapID_temp.submap_index + submap_data_.num_indices(0);
      }
      else
      {
          i = submapID_temp.submap_index;
      }

      rtree.insert(std::make_pair(b, i));
  }
  //*****zz 19-10-14*****//

  //*****zz 19-10-14*****//no SPA
  if(!options_.use_odom()){
    AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(trajectory_id, insertion_submaps,
                              newly_finished_submap);
    });
  }
  //*****zz 19-10-14*****//
}

void SparsePoseGraph::AddWorkItem(const std::function<void()>& work_item) {
  if (work_queue_ == nullptr) {
    work_item();
  } else {
    work_queue_->push_back(work_item);
  }
}

void SparsePoseGraph::AddImuData(const int trajectory_id,
                                 const sensor::ImuData& imu_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddImuData(trajectory_id, imu_data);
  });
}

void SparsePoseGraph::AddOdometerData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddOdometerData(trajectory_id, odometry_data);
  });
}

void SparsePoseGraph::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  LOG(FATAL) << "Not yet implemented for 2D.";
}

void SparsePoseGraph::ComputeConstraint(const mapping::NodeId& node_id,
                                        const mapping::SubmapId& submap_id) {
  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  const common::Time scan_time = GetLatestScanTime(node_id, submap_id);
  const common::Time last_connection_time =
      trajectory_connectivity_state_.LastConnectionTime(
          node_id.trajectory_id, submap_id.trajectory_id);
  if (node_id.trajectory_id == submap_id.trajectory_id ||
      scan_time <
          last_connection_time +
              common::FromSeconds(
                  options_.global_constraint_search_after_n_seconds())) {
    // If the scan and the submap belong to the same trajectory or if there has
    // been a recent global constraint that ties that scan's trajectory to the
    // submap's trajectory, it suffices to do a match constrained to a local
    // search window.
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_.submap_data()
            .at(submap_id.trajectory_id)
            .at(submap_id.submap_index)
            .pose.inverse() *
        optimization_problem_.node_data()
            .at(node_id.trajectory_id)
            .at(node_id.node_index)
            .pose;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get(),
        initial_relative_pose);
  } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get());
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(
    const mapping::SubmapId& submap_id) {
  const auto& submap_data = submap_data_.at(submap_id);
  const auto& node_data = optimization_problem_.node_data();
  for (size_t trajectory_id = 0; trajectory_id != node_data.size();
       ++trajectory_id) {
    for (const auto& index_node_data : node_data[trajectory_id]) {
      const mapping::NodeId node_id{static_cast<int>(trajectory_id),
                                    index_node_data.first};
      CHECK(!trajectory_nodes_.at(node_id).trimmed());
      if (submap_data.node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, submap_id);
      }
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForScan(
    const int trajectory_id,
    std::vector<std::shared_ptr<const Submap>> insertion_submaps,
    const bool newly_finished_submap) {
  const std::vector<mapping::SubmapId> submap_ids =
      GrowSubmapTransformsAsNeeded(trajectory_id, insertion_submaps);
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  const mapping::SubmapId matching_id = submap_ids.front();
  const mapping::NodeId node_id{
      matching_id.trajectory_id,
      static_cast<size_t>(matching_id.trajectory_id) <
          optimization_problem_.node_data().size() &&
          !optimization_problem_.node_data()[matching_id.trajectory_id]
              .empty()
      ? static_cast<int>(optimization_problem_.node_data()
          .at(matching_id.trajectory_id)
          .rbegin()
          ->first +
          1)
      : 0};
  const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
  const transform::Rigid2d pose = transform::Project2D(
      constant_data->initial_pose *
      transform::Rigid3d::Rotation(constant_data->gravity_alignment.inverse()));
  const transform::Rigid2d optimized_pose =
      optimization_problem_.submap_data()
          .at(matching_id.trajectory_id)
          .at(matching_id.submap_index)
          .pose *
      sparse_pose_graph::ComputeSubmapPose(*insertion_submaps.front())
          .inverse() *
      pose;
  optimization_problem_.AddTrajectoryNode(
      matching_id.trajectory_id, constant_data->time, pose, optimized_pose,
      constant_data->gravity_alignment);
  for (size_t i = 0; i < insertion_submaps.size(); ++i) {
    const mapping::SubmapId submap_id = submap_ids[i];
    // Even if this was the last scan added to 'submap_id', the submap will only
    // be marked as finished in 'submap_data_' further below.
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid2d constraint_transform =
        sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
        pose;
    constraints_.push_back(Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                       options_.matcher_translation_weight(),
                                       options_.matcher_rotation_weight()},
                                      Constraint::INTRA_SUBMAP});
  }

  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      const mapping::SubmapId submap_id{trajectory_id, submap_index};
      if (submap_data_.at(submap_id).state == SubmapState::kFinished) {
        CHECK_EQ(submap_data_.at(submap_id).node_ids.count(node_id), 0);
        ComputeConstraint(node_id, submap_id);
      }
    }
  }

  if (newly_finished_submap) {
    const mapping::SubmapId finished_submap_id = submap_ids.front();
    SubmapData& finished_submap_data = submap_data_.at(finished_submap_id);
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old scans.
    ComputeConstraintsForOldScans(finished_submap_id);
  }
  constraint_builder_.NotifyEndOfScan();
  ++num_scans_since_last_loop_closure_;
  if (options_.optimize_every_n_scans() > 0 &&
      num_scans_since_last_loop_closure_ > options_.optimize_every_n_scans()) {
    CHECK(!run_loop_closure_);
    run_loop_closure_ = true;
    // If there is a 'work_queue_' already, some other thread will take care.
    if (work_queue_ == nullptr) {
      work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
      HandleWorkQueue();
    }
  }
}

common::Time SparsePoseGraph::GetLatestScanTime(
    const mapping::NodeId& node_id, const mapping::SubmapId& submap_id) const {
  common::Time time = trajectory_nodes_.at(node_id).constant_data->time;
  const SubmapData& submap_data = submap_data_.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    const mapping::NodeId last_submap_node_id =
        *submap_data_.at(submap_id).node_ids.rbegin();
    time = std::max(
        time, trajectory_nodes_.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void SparsePoseGraph::UpdateTrajectoryConnectivity(
    const sparse_pose_graph::ConstraintBuilder::Result& result) {
  for (const Constraint& constraint : result) {
    CHECK_EQ(constraint.tag,
             mapping::SparsePoseGraph::Constraint::INTER_SUBMAP);
    const common::Time time =
        GetLatestScanTime(constraint.node_id, constraint.submap_id);
    trajectory_connectivity_state_.Connect(constraint.node_id.trajectory_id,
                                           constraint.submap_id.trajectory_id,
                                           time);
  }
}

void SparsePoseGraph::HandleWorkQueue() {
  constraint_builder_.WhenDone(
      [this](const sparse_pose_graph::ConstraintBuilder::Result& result) {
        {
          common::MutexLocker locker(&mutex_);
          constraints_.insert(constraints_.end(), result.begin(), result.end());
        }
        RunOptimization();

        common::MutexLocker locker(&mutex_);
        UpdateTrajectoryConnectivity(result);
        TrimmingHandle trimming_handle(this);
        for (auto& trimmer : trimmers_) {
          trimmer->Trim(&trimming_handle);
        }

        num_scans_since_last_loop_closure_ = 0;
        run_loop_closure_ = false;
        while (!run_loop_closure_) {
          if (work_queue_->empty()) {
            work_queue_.reset();
            return;
          }
          work_queue_->front()();
          work_queue_->pop_front();
        }
        LOG(INFO) << "Remaining work items in queue: " << work_queue_->size();
        // We have to optimize again.
        HandleWorkQueue();
      });
}

void SparsePoseGraph::WaitForAllComputations() {
  bool notification = false;
  common::MutexLocker locker(&mutex_);
  const int num_finished_scans_at_start =
      constraint_builder_.GetNumFinishedScans();
  while (!locker.AwaitWithTimeout(
      [this]() REQUIRES(mutex_) {
        return constraint_builder_.GetNumFinishedScans() ==
               num_trajectory_nodes_;
      },
      common::FromSeconds(1.))) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. *
                         (constraint_builder_.GetNumFinishedScans() -
                          num_finished_scans_at_start) /
                         (num_trajectory_nodes_ - num_finished_scans_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone(
      [this, &notification](
          const sparse_pose_graph::ConstraintBuilder::Result& result) {
        common::MutexLocker locker(&mutex_);
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        notification = true;
      });
  locker.Await([&notification]() { return notification; });
}

void SparsePoseGraph::FreezeTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  trajectory_connectivity_state_.Add(trajectory_id);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

void SparsePoseGraph::AddSubmapFromProto(const int trajectory_id,
                                         const transform::Rigid3d& initial_pose,
                                         const mapping::proto::Submap& submap) {
  if (!submap.has_submap_2d()) {
    return;
  }

  std::shared_ptr<const Submap> submap_ptr =
      std::make_shared<const Submap>(submap.submap_2d());
  const transform::Rigid2d initial_pose_2d = transform::Project2D(initial_pose);

  common::MutexLocker locker(&mutex_);
  trajectory_connectivity_state_.Add(trajectory_id);
  const mapping::SubmapId submap_id =
      submap_data_.Append(trajectory_id, SubmapData());
  submap_data_.at(submap_id).submap = submap_ptr;
  // Immediately show the submap at the optimized pose.
  CHECK_GE(static_cast<size_t>(submap_data_.num_trajectories()),
           optimized_submap_transforms_.size());
  optimized_submap_transforms_.resize(submap_data_.num_trajectories());
  CHECK_EQ(optimized_submap_transforms_.at(trajectory_id).size(),
           submap_id.submap_index);
  optimized_submap_transforms_.at(trajectory_id)
      .emplace(submap_id.submap_index,
               sparse_pose_graph::SubmapData{initial_pose_2d});
  AddWorkItem([this, submap_id, initial_pose_2d]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(submap_id.trajectory_id), 1);
    submap_data_.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_.AddSubmap(submap_id.trajectory_id, initial_pose_2d);
  });

  //*****zz 2019-10-14*****//
  //add r-tree information from pbstream
  Eigen::Vector2d maxgrid = submap_data_.at(submap_id).submap->probability_grid().limits().max();
  double maxx= maxgrid.x();
  double maxy= maxgrid.y();
  double limitx = submap_data_.at(submap_id).submap->probability_grid().limits().cell_limits().num_x_cells;
  double limity = submap_data_.at(submap_id).submap->probability_grid().limits().cell_limits().num_y_cells;
  double originx,originy;//zz
  originx = submap_data_.at(submap_id).submap->local_pose().translation().x();
  originy = submap_data_.at(submap_id).submap->local_pose().translation().y();

  //LOG(INFO)<<originx<<","<<originy<<endl;
  //add into R-tree
  double reso = submap_data_.at(submap_id).submap->probability_grid().limits().resolution();
  double minx,miny;
  minx= maxx - reso*limity;
  miny= maxy - reso*limitx;

  box b(point(minx, miny), point(maxx, maxy));
  unsigned i = submap_id.submap_index;
  rtree.insert(std::make_pair(b, i));
  //*****zz 2019-10-14*****//

}


// ----- JuRan HuangTengfei 2021-11-24 ----- // 
void SparsePoseGraph::BuildMapImageFromSubmaps(const std::string& map_save_path_, const std::string& map_info_save_path_, double reso) {
    auto all_submap = GetAllSubmapData();
    double historical_max_x = -INFINITY;
    double historical_min_x = INFINITY;
    double historical_max_y = -INFINITY;
    double historical_min_y = INFINITY;
    std::fstream map_info_writer(map_info_save_path_, std::ios::out);

    std::cout << all_submap[0].size() << " in historical map" <<std::endl;
    for(int i = 0;i < all_submap[0].size();i++){
        mapping::SubmapId submapID_temp{0,i};
        ProbabilityGrid probability_grid_temp = submap_data_.at(submapID_temp).submap->probability_grid();
        Eigen::Vector2d maxgrid = probability_grid_temp.limits().max();
        double limitx = probability_grid_temp.limits().cell_limits().num_x_cells;
        double limity = probability_grid_temp.limits().cell_limits().num_y_cells;
        double minx, miny;
        double maxx= maxgrid.x();
        double maxy= maxgrid.y();
        minx= maxx - reso*limity;
        miny= maxy - reso*limitx;

        if(maxx > historical_max_x){
            historical_max_x = maxx;
        }
        if(maxy > historical_max_y){
            historical_max_y = maxy;
        }
        if(minx < historical_min_x){
            historical_min_x = minx;
        }
        if(miny < historical_min_y){
            historical_min_y = miny;
        }
    }

    std::cout << "the board of historical map:" <<std::endl;
    std::cout << "min x: " << historical_min_x << std::endl;
    std::cout << "min y: " << historical_min_y << std::endl;
    std::cout << "max x: " << historical_max_x << std::endl;
    std::cout << "max y: " << historical_max_y << std::endl << std::endl;

    std::vector<std::vector<double>> map;
    std::vector<std::vector<int>> count;

    int map_board_x = (historical_max_x - historical_min_x)/reso + 0.5;
    int map_board_y = (historical_max_y - historical_min_y)/reso + 0.5;

    std::cout << "the size of historical cvMat" << std::endl;
    std::cout << "map board x: " << map_board_x <<std::endl;
    std::cout << "map board y: " << map_board_y <<std::endl << std::endl;

    double orgin_in_map_x = (0 - historical_min_x)/reso;
    double orgin_in_map_y = (0 - historical_min_y)/reso;

    std::cout << "the orgin in historical cvMat" << std::endl;
    std::cout << "orgin in map x: " << orgin_in_map_x <<std::endl;
    std::cout << "orgin in map y: " << orgin_in_map_y <<std::endl << std::endl;
    map_info_writer << orgin_in_map_x << " " << orgin_in_map_y << " " << reso;
    map_info_writer.close();

    map.resize(map_board_x);
    for(int i = 0;i<map_board_x;i++){
        map[i].resize(map_board_y, 0);
    }

    count.resize(map_board_x);
    for(int i = 0;i<map_board_x;i++){
        count[i].resize(map_board_y, 0);
    }

    std::cout << "start fusing historical map ..." << std::endl;
    for(int i = 0;i < all_submap[0].size();i++){
        mapping::SubmapId submapID_temp{0,i};
        SubmapData& one_submap = submap_data_.at(submapID_temp);

        ProbabilityGrid probability_grid_temp = one_submap.submap->probability_grid();
        Eigen::Vector2d maxgrid = probability_grid_temp.limits().max();
        double limitx = probability_grid_temp.limits().cell_limits().num_x_cells;
        double limity = probability_grid_temp.limits().cell_limits().num_y_cells;
        double minx, miny;
        double maxx= maxgrid.x();
        double maxy= maxgrid.y();
        minx= maxx - reso*limity;
        miny= maxy - reso*limitx;

        for (double m = minx;m < maxx;)
        {
            for (double n = miny; n < maxy;)
            {
                Eigen::Vector2f point(m,n);
                Eigen::Array2i cell_index = probability_grid_temp.limits().GetCellIndex(point);

                if(probability_grid_temp.limits().Contains(cell_index) == true)
                {
                    float probility = probability_grid_temp.GetProbability(cell_index);
                    //get average above ths
                    if(probility >= 0.7)
                    {
                        int index_in_map_x = (m - historical_min_x)/reso;
                        int index_in_map_y = (n - historical_min_y)/reso;

                        count[index_in_map_x][index_in_map_y] += 1;
                    }
                }
                n+=reso;
            }
            m+=reso;
        }
    }

    std::cout << "successfully fusing historical map" << std::endl;

    cv::Mat cv_map(map_board_x, map_board_y, CV_8UC1, Scalar(0));

    //get average above ths
    for (int m = 0;m < map_board_x;m++)
        for (int n=0; n < map_board_y; n++)
        {
            if(count[m][n]!=0)
            {
                cv_map.at<uchar>(m,n) = 255;
            }
        }

    cv::imwrite(map_save_path_, cv_map);

    std::cout << "Save map to " << map_save_path_ << std::endl;
}


void SparsePoseGraph::UpdateSubmapsFromMapImage(const std::string& original_map_img_path_,
                                                const std::string& modified_map_img_path_,
                                                const std::string& map_info_path_){
    double orgin_x_in_map, orgin_y_in_map;
    double map_resolution;

    std::ifstream map_set_file(map_info_path_);
    if(!map_set_file.is_open()){
        std::cerr << "Fail to load map info txt file" << std::endl;
        exit(-1);
    }
    map_set_file >> orgin_x_in_map >> orgin_y_in_map >> map_resolution;

    cv::Mat erased, added;
    {   cv::Mat original_map = cv::imread(original_map_img_path_, cv::IMREAD_GRAYSCALE);
        cv::Mat modified_map = cv::imread(modified_map_img_path_, cv::IMREAD_GRAYSCALE);
        if(original_map.data == nullptr){
            std::cerr << "Fail to load map from " << original_map_img_path_ << std::endl;
            exit(-1);
        }
        if(modified_map.data == nullptr){
            std::cerr << "Fail to load map from " << modified_map_img_path_ << std::endl;
            exit(-1);
        }
        cv::subtract(original_map, modified_map, erased);
        cv::subtract(modified_map, original_map, added);
    }

    auto all_submap = GetAllSubmapData();

    std::cout << all_submap[0].size() << " in historical map" <<std::endl;

    std::cout << "start modify historical map ..." << std::endl;

    for(int i = 0;i < all_submap[0].size();i++){
        mapping::SubmapId submapID_temp{0,i};
        SubmapData& one_submap = submap_data_.at(submapID_temp);

        const ProbabilityGrid* const_probability_grid_ptr =
                &one_submap.submap->probability_grid();
        Eigen::Vector2d maxgrid = const_probability_grid_ptr->limits().max();
        double limitx = const_probability_grid_ptr->limits().cell_limits().num_x_cells;
        double limity = const_probability_grid_ptr->limits().cell_limits().num_y_cells;
        double minx, miny;
        double maxx = maxgrid.x();
        double maxy = maxgrid.y();
        minx = maxx - map_resolution * limity;
        miny = maxy - map_resolution * limitx;

        for (double m = minx;m < maxx;)
        {
            for (double n = miny; n < maxy;)
            {
                Eigen::Vector2f point(m, n);
                Eigen::Array2i cell_index =
                        const_probability_grid_ptr->limits().GetCellIndex(point);

                if(const_probability_grid_ptr->limits().Contains(cell_index) == true)
                {
                    float probility = const_probability_grid_ptr->GetProbability(cell_index);

                    int index_in_map_x = m/map_resolution + orgin_x_in_map;
                    int index_in_map_y = n/map_resolution + orgin_y_in_map;

                    if(index_in_map_x > added.rows || index_in_map_x < 0
                    || index_in_map_y > added.cols || index_in_map_y < 0)
                        continue;

                    if(erased.at<uchar>(index_in_map_x, index_in_map_y) > 128)
                    {
                        ProbabilityGrid* probability_grid_ptr =
                                const_cast<ProbabilityGrid*>(const_probability_grid_ptr);
                        probability_grid_ptr->ChangeProbability(cell_index, 0.1);
                    }
                    else if(added.at<uchar>(index_in_map_x, index_in_map_y) > 128)
                    {
                        ProbabilityGrid* probability_grid_ptr =
                                const_cast<ProbabilityGrid*>(const_probability_grid_ptr);
                        probability_grid_ptr->ChangeProbability(cell_index, 0.7);
                    }
                }
                n += map_resolution;
            }
            m += map_resolution;
        }
    }
    std::cout << "successfully modify historical map" << std::endl;
}
// ----- JuRan HuangTengfei 2021-11-24 ----- //


void SparsePoseGraph::AddTrimmer(
    std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) {
  common::MutexLocker locker(&mutex_);
  // C++11 does not allow us to move a unique_ptr into a lambda.
  mapping::PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  AddWorkItem([this, trimmer_ptr]()
                  REQUIRES(mutex_) { trimmers_.emplace_back(trimmer_ptr); });
}

void SparsePoseGraph::RunFinalOptimization() {
  WaitForAllComputations();
  optimization_problem_.SetMaxNumIterations(
      options_.max_num_final_iterations());
  RunOptimization();
  optimization_problem_.SetMaxNumIterations(
      options_.optimization_problem_options()
          .ceres_solver_options()
          .max_num_iterations());
}

void SparsePoseGraph::RunOptimization() {
  if (optimization_problem_.submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_, constraints_ and
  // frozen_trajectories_ when executing the Solve. Solve is time consuming, so
  // not taking the mutex before Solve to avoid blocking foreground processing.
  optimization_problem_.Solve(constraints_, frozen_trajectories_);
  common::MutexLocker locker(&mutex_);

  const auto& submap_data = optimization_problem_.submap_data();
  const auto& node_data = optimization_problem_.node_data();
  for (int trajectory_id = 0;
       trajectory_id != static_cast<int>(node_data.size()); ++trajectory_id) {
    const int num_nodes = trajectory_nodes_.num_indices(trajectory_id);
    for (const auto& node_data_index : node_data.at(trajectory_id)) {
      const mapping::NodeId node_id{trajectory_id, node_data_index.first};
      auto& node = trajectory_nodes_.at(node_id);
      node.pose =
          transform::Embed3D(node_data_index.second.pose) *
          transform::Rigid3d::Rotation(node.constant_data->gravity_alignment);
    }
    // Extrapolate all point cloud poses that were added later.
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        optimized_submap_transforms_, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();
    int last_optimized_node_index =
        node_data.at(trajectory_id).empty()
            ? 0
            : node_data.at(trajectory_id).rbegin()->first;
    for (int node_index = last_optimized_node_index + 1; node_index < num_nodes;
         ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      auto& node_pose = trajectory_nodes_.at(node_id).pose;
      node_pose = old_global_to_new_global * node_pose;
    }
  }
  optimized_submap_transforms_ = submap_data;
}

std::vector<std::vector<mapping::TrajectoryNode>>
SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_.data();
}

std::vector<SparsePoseGraph::Constraint> SparsePoseGraph::constraints() {
  std::vector<Constraint> result;
  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : constraints_) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 trajectory_nodes_.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

transform::Rigid3d SparsePoseGraph::GetLocalToGlobalTransform(
    const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  return ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                       trajectory_id);
}

std::vector<std::vector<int>> SparsePoseGraph::GetConnectedTrajectories() {
  return trajectory_connectivity_state_.Components();
}

int SparsePoseGraph::num_submaps(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  if (trajectory_id >= submap_data_.num_trajectories()) {
    return 0;
  }
  return submap_data_.num_indices(trajectory_id);
}

//*****zz 19-10-14*****//
Mat SparsePoseGraph::GetImage(double reckonx,double reckony,double yaw, int rows, int cols, int cen_row, int cen_col, float reso ,int search_num ,float threshold)
{
    std::vector<value> result_temp;
    rtree.query(bgi::nearest(point(reckonx, reckony), search_num), std::back_inserter(result_temp));

    // find values intersecting some area defined by a box
//    float center_x = reckonx + 100 * reso *cos(yaw);
//    float center_y = reckony + 100 * reso *sin(yaw);
//    box query_box(point(center_x - 200*reso, center_y - 200*reso), point(center_x + 200*reso, center_y + 200*reso));
//    rtree.query(bgi::intersects(query_box), std::back_inserter(result_temp));

    //To vector int
    result_n.clear();
    BOOST_FOREACH(value const& v, result_temp)
    {
       int temp = v .second;
       result_n.push_back(temp);
    }
        cv::Mat currunt_map(rows,cols,CV_16UC1,Scalar(0));
        cv::Mat count(rows,cols,CV_8UC1,Scalar(0));
	cv::Mat first_flag(TiEV::GRID_ROW,TiEV::GRID_COL,CV_8UC1,Scalar(0));//test min/max probility
        double max_x = cen_row*reso;
        double max_y = cen_col*reso;
        double left_top_x = reckonx + max_x * cos(yaw) - max_y * sin(yaw);
        double left_top_y = reckony + max_x * sin(yaw) + max_y * cos(yaw);

        Eigen::Vector2f point;

        int countnum = 0;
        for(int i=0;i<result_n.size();i++)
        {
            mapping::SubmapId submapID_temp{0,0};
            if(result_n[i]<submap_data_.num_indices(0))
            {
                submapID_temp.trajectory_id=0;
                submapID_temp.submap_index=result_n[i];
            }
            else
            {
                submapID_temp.trajectory_id=1;
                submapID_temp.submap_index = result_n[i]-submap_data_.num_indices(0);
            }


          if(submap_data_.at(submapID_temp).submap != NULL)
           {
            if(submap_data_.at(submapID_temp).submap->finished() )
            // && TrimJudge(submapID_temp.submap_index)==false)     (submapID_temp.submap_index - submapID_helper.submap_index) < 3
           {
            countnum += 1;
            //LOG(INFO)<<"current_ID:"<<submapID_helper<<";query nearstID"<<submapID_temp<<"dist:"<<dist<<endl<<endl;
            ProbabilityGrid probability_grid_temp = submap_data_.at(submapID_temp).submap->probability_grid();
            Eigen::Vector2d maxgrid = probability_grid_temp.limits().max();
            double limitx = probability_grid_temp.limits().cell_limits().num_x_cells;
            double limity = probability_grid_temp.limits().cell_limits().num_y_cells;
            double minx, miny;
            double maxx= maxgrid.x();
            double maxy= maxgrid.y();
            minx= maxx - reso*limity;
            miny= maxy - reso*limitx;

            for (int m = 0;m < currunt_map.rows;m++)
            {
                for (int n=0; n < currunt_map.cols; n++)
                {
                    double min_x = -m * reso;
                    double min_y = -n * reso;
                    point.x() = left_top_x + min_x * cos(yaw) - min_y * sin(yaw);
                    point.y() = left_top_y + min_x * sin(yaw) + min_y * cos(yaw);
                    if( minx<point.x() && point.x()<maxx && miny<point.y() && point.y()<maxy)
                    {
                        Eigen::Array2i cell_index = probability_grid_temp.limits().GetCellIndex(point);
                        if(probability_grid_temp.limits().Contains(cell_index) == true)
                        {
                            float probility = probability_grid_temp.GetProbability(cell_index);
                            int value_temp = (probility-0.1f)/0.8f * 255 ;
                            uint16 value = mapping::ProbabilityToValue(probility);
                            ///power average
//                            if(value_temp>=0)
//                            {
//                              int power = 0;
//                              if(submapID_temp.submap_index >= 217)
//                              {
//                                power = search_num;
//                              }
//                              else
//                              {
//                                power = 1;
//                              }
//                              currunt_map.ptr<ushort>(m)[n] += power * value_temp;
//                              count.ptr<uchar>(m)[n] += power;
//                            }


			    //get average above ths
                            if(probility != 0.1f && probility >= threshold )//&& probility>0.2f)
                            {
                               currunt_map.ptr<ushort>(m)[n] += value_temp;
                               //currunt_map.at<int>(m,n) += value;
                               count.ptr<uchar>(m)[n] += 1;
                            }

                            //get average_total
//                            if(probility != 1.0f)// >= 2.0f??
//                            {
//                               currunt_map.at<ushort>(m,n) += value_temp;
//                               //currunt_map.at<int>(m,n) += value;
//                               count.at<uchar>(m,n)+=1;
//                            }

/*
                            //get min probility
                            if(probility != 1.0f && probility >= threshold)
                            {
                               if(first_flag.at<uchar>(m,n)==0)
                               {
                                   currunt_map.at<ushort>(m,n) = value_temp;
                                   first_flag.at<uchar>(m,n)=1;
                               }
                               else
                               {
                                   if(currunt_map.at<ushort>(m,n) < value_temp)
                                   {
                                       currunt_map.at<ushort>(m,n) = value_temp;
                                   }
                               }
                            }
*/
                        }
                    }
                }
            }
           }
        }

        }
        //LOG(INFO)<<"current_ID:"<<submapID_helper<<";mulitipic num"<<countnum<<endl;
        std::stringstream ss;
	//get average above ths
        for (int m = 0;m < currunt_map.rows;m++)
            for (int n=0; n < currunt_map.cols; n++)
            {
                if(count.at<uchar>(m,n)!=0)
                {
                    currunt_map.at<ushort>(m,n) = currunt_map.at<ushort>(m,n)/count.at<uchar>(m,n);
                }
            }

        //get average  delete probility below ths
//        for (int m = 0;m < currunt_map.rows;m++)
//            for (int n=0; n < currunt_map.cols; n++)
//            {
//                if(count.at<uchar>(m,n)!=0)
//                {
//                    currunt_map.at<ushort>(m,n) = currunt_map.at<ushort>(m,n)/count.at<uchar>(m,n);
//                    int value_ths = (threshold-0.1f)/0.8f * 255 ;
//                    if(currunt_map.at<ushort>(m,n) < value_ths)
//                    {
//                        currunt_map.at<ushort>(m,n) = 0;
//                    }
//                }
//            }
        cv::Mat MatTemp2;
        currunt_map.convertTo(MatTemp2,CV_8UC1);
        return MatTemp2;
}
//*****zz 19-10-14*****//

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapData(
    const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
SparsePoseGraph::GetAllSubmapData() {
  common::MutexLocker locker(&mutex_);
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
      all_submap_data(submap_data_.num_trajectories());
  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    all_submap_data[trajectory_id].reserve(
        submap_data_.num_indices(trajectory_id));
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      all_submap_data[trajectory_id].emplace_back(GetSubmapDataUnderLock(
          mapping::SubmapId{trajectory_id, submap_index}));
    }
  }
  return all_submap_data;
}

transform::Rigid3d SparsePoseGraph::ComputeLocalToGlobalTransform(
    const std::vector<std::map<int, sparse_pose_graph::SubmapData>>&
        submap_transforms,
    const int trajectory_id) const {
  if (trajectory_id >= static_cast<int>(submap_transforms.size()) ||
      submap_transforms.at(trajectory_id).empty()) {
    return transform::Rigid3d::Identity();
  }

  const int submap_index = submap_transforms.at(trajectory_id).rbegin()->first;
  const mapping::SubmapId last_optimized_submap_id{trajectory_id, submap_index};
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return transform::Embed3D(
             submap_transforms.at(trajectory_id).at(submap_index).pose) *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapDataUnderLock(
    const mapping::SubmapId& submap_id) {
  if (submap_data_.at(submap_id).state == SubmapState::kTrimmed) {
    return {};
  }
  auto submap = submap_data_.at(submap_id).submap;
  if (submap_id.trajectory_id <
          static_cast<int>(optimized_submap_transforms_.size()) &&
      submap_id.submap_index < static_cast<int>(optimized_submap_transforms_
                                                    .at(submap_id.trajectory_id)
                                                    .size())) {
    // We already have an optimized pose.
    return {submap, transform::Embed3D(
                        optimized_submap_transforms_.at(submap_id.trajectory_id)
                            .at(submap_id.submap_index)
                            .pose)};
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

SparsePoseGraph::TrimmingHandle::TrimmingHandle(SparsePoseGraph* const parent)
    : parent_(parent) {}

int SparsePoseGraph::TrimmingHandle::num_submaps(
    const int trajectory_id) const {
  return parent_->optimization_problem_.submap_data().at(trajectory_id).size();
}

void SparsePoseGraph::TrimmingHandle::MarkSubmapAsTrimmed(
    const mapping::SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  std::set<mapping::NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }
  // Remove all 'constraints_' related to 'submap_id'.
  std::set<mapping::NodeId> nodes_to_remove;
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (constraint.submap_id == submap_id) {
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
            nodes_to_retain.count(constraint.node_id) == 0) {
          // This node will no longer be INTRA_SUBMAP contrained and has to be
          // removed.
          nodes_to_remove.insert(constraint.node_id);
        }
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }
  // Remove all 'constraints_' related to 'nodes_to_remove'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  auto& submap_data = parent_->submap_data_.at(submap_id);
  CHECK(submap_data.state == SubmapState::kFinished);
  submap_data.state = SubmapState::kTrimmed;
  CHECK(submap_data.submap != nullptr);
  submap_data.submap.reset();
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_.TrimSubmap(submap_id);

  // Mark the 'nodes_to_remove' as trimmed and remove their data.
  for (const mapping::NodeId& node_id : nodes_to_remove) {
    CHECK(!parent_->trajectory_nodes_.at(node_id).trimmed());
    parent_->trajectory_nodes_.at(node_id).constant_data.reset();
    parent_->optimization_problem_.TrimTrajectoryNode(node_id);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
