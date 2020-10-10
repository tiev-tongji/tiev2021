#pragma once
#include <vector>
#include <unordered_map>
#include <list>
#include <Eigen/Core>
#include <Eigen/Geometry>

class DataAssociation
{
private:
  // double getDistance(const geometry_msgs::Point &measurement,
  //                    const geometry_msgs::Point &tracker);
  const double score_threshold_;

public:
  DataAssociation();
  bool assign(const Eigen::MatrixXd &src,
              std::unordered_map<int, int> &direct_assignment,
              std::unordered_map<int, int> &reverse_assignment);
  // Eigen::MatrixXd calcScoreMatrix(const autoware_msgs::DynamicObjectWithFeatureArray &measurements,
  //                                 const std::list<std::shared_ptr<Tracker>> &trackers);
  virtual ~DataAssociation(){};
};
