#include "speed_data.h"
#include <algorithm>

namespace TiEV {

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
                    : std::vector<SpeedPoint>(std::move(speed_points)) {
    std::sort(begin(), end(), [](const SpeedPoint& sp1, const SpeedPoint& sp2) {
       return sp1.t() < sp2.t();
    });
}

bool SpeedData::AppendSpeedPoint(const double s, const double tt, const double v,
                                 const double a, const double da) {
    push_back(SpeedPoint(s, tt , v, a, da));
    return true;
}

double SpeedData::TotalTime() const {
    return back().t() - front().t();
}

double SpeedData::TotalLength() const {
    return back().s() - front().s();
}

double SpeedData::GetSByTime(double time) const {
    if (size() < 2) return front().s();

    auto comp = [](const SpeedPoint point, const double t) {
        return point.t() < t;
    };

    auto itr = std::lower_bound(begin(), end(), time, comp);
    if (itr == begin()) {
        return (*begin()).s();
    } else if (itr == end()) {
        return (*rbegin()).s();
    }

    return lerp((*(itr - 1)).s(), (*(itr - 1)).t(),
                (*itr).s(), (*itr).t(), time);    
}

double SpeedData::GetVelocityByS(double s) const {
    auto comp = [](const SpeedPoint point, const double s) {
        return point.s() < s;
    };

    auto itr = std::lower_bound(begin(), end(), s, comp);
    if (itr == begin()) {
        return (*begin()).v();
    } else if (itr == end()) {
        return (*rbegin()).v();
    }

    return lerp((*(itr - 1)).v(), (*(itr - 1)).s(),
                (*itr).v(), (*itr).s(), s);
}

double SpeedData::GetAccelByS(double s) const {
    auto comp = [](const SpeedPoint point, const double s) {
        return point.s() < s;
    };

    auto itr = std::lower_bound(begin(), end(), s, comp);
    if (itr == begin()) {
        return (*begin()).a();
    } else if (itr == end()) {
        return (*rbegin()).a();
    }

    return lerp((*(itr - 1)).a(), (*(itr - 1)).s(),
                (*itr).a(), (*itr).s(), s);
}

double SpeedData::GetTimeByS(double s) const {
    auto comp = [](const SpeedPoint point, const double s) {
        return point.s() < s;
    };

    auto itr = std::lower_bound(begin(), end(), s, comp);
    if (itr == begin()) {
        return (*begin()).t();
    } else if (itr == end()) {
        return (*rbegin()).t();
    }

    return lerp((*(itr - 1)).t(), (*(itr - 1)).s(),
                (*itr).t(), (*itr).s(), s);
}

} // namespace TiEV
