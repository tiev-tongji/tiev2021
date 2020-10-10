/*
 * Copyright (c) 2018 TiEV (Tongji Intelligent Electric Vehicle).
 *
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <vector>
#include <utility>

namespace TiEV {

class Smoother{
public:
    std::vector<std::pair<double, double>> quadricSmoother(
        const std::vector<std::pair<double, double>> &pointlist,
        const double &weight_data = 1.0,
        const double &weight_smooth = 0.1,
        const double &tolerance = 0.00001);
};

}
