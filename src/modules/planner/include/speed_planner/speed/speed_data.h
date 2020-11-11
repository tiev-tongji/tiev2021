#pragma once

#include <vector>
#include "speed_point.h"
#include "../math/linear_interpolation.h"

namespace TiEV {

class SpeedData : public std::vector<SpeedPoint> {
public:
    SpeedData() = default;

    virtual ~SpeedData() = default;

    explicit SpeedData(std::vector<SpeedPoint> speed_points);

    bool AppendSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da);

    double TotalTime() const;

    double TotalLength() const;

    double GetSByTime(double time) const;

    double GetVelocityByS(double s) const;

    double GetAccelByS(double s) const;

    double GetTimeByS(double s) const;
};
}
