//
// Created by xlz on 17-10-1.
//用来保存数据间的变换矩阵

#ifndef TEST_CARTOGRAPHER_TF_EIGEN_H
#define TEST_CARTOGRAPHER_TF_EIGEN_H
#include "cartographer/transform/rigid_transform.h"
using cartographer::transform::Rigid3d;
#include "msg.h"
#include <iostream>
#include <tiff.h>
#include <time.h>
#include <sstream>

#include <iomanip>
#include <fstream>
#include <vector>
#include "cartographer/common/time.h"

using ::cartographer::transform::Rigid3d;
using namespace std;
//AddRangefinderData 要const Eigen::Vector3f& origin,  origin规则记在笔记本最后面
//TransformPointCloud 要const transform::Rigid3f& transform
class tf_eigen {
    public:
};



#endif //TEST_CARTOGRAPHER_TF_EIGEN_H
