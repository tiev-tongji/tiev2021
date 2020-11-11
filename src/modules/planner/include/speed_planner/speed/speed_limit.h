#pragma once

#include <vector>
#include <algorithm>
#include "../math/linear_interpolation.h"

namespace TiEV {

class SpeedLimit : public std::vector<std::pair<double, double> > {
public:
    SpeedLimit() = default;

    virtual ~SpeedLimit() = default;

    explicit SpeedLimit(std::vector<std::pair<double, double> > speed_limit);

    double GetSpeedLimit(double s) const;
};
}