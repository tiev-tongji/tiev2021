#ifndef ROTATE_H
#define ROTATE_H

#include "common.h"
#include "cluster.h"

/**
* @brief detect angle of lane lines, align vertically the lane mask for clustering, and convert the point back to
* original image coordinate.
*/
class ImageAlign {
public:
    ImageAlign(cv::Mat feature_mask, int single_side_widen_width = 0);
    bool ok() const {return yaw_ok;}
    double get_yaw() {return yaw_ok ? yaw : 0.0;}
    void set_yaw(double p_yaw);
    int align(cv::InputArray, cv::OutputArray);
    int revert(cv::InputArray, cv::OutputArray);
    int align(const cluster::Cluster_<float>& in, cluster::Cluster_<float>& out);
    int revert(const cluster::Cluster_<float>& in, cluster::Cluster_<float>& out);
    int align(const cv::Point2f& in_point, cv::Point2f& out_point);
    int revert(const cv::Point2f& in_point, cv::Point2f& out_point);


private:
    int rows;
    int cols;
    int single_side_widen_width;
    double yaw;
    bool yaw_ok;
    cv::Mat affine_matrix;
    cv::Mat affine_matrix_inv;
};

#endif // ROTATE_H
