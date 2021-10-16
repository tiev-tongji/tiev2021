#include "speed_limit.h"

#include <iostream>

namespace TiEV {

SpeedLimit::SpeedLimit(std::vector<std::pair<double, double>> speed_limit)
    : std::vector<std::pair<double, double>>(speed_limit) {}

double SpeedLimit::GetSpeedLimit(double s) const {
  auto comp = [](const std::pair<double, double> point, const double s) {
    return point.first < s;
  };

  auto itr = std::lower_bound(begin(), end(), s, comp);
  if (itr == begin()) {
    return (*begin()).second;
  } else if (itr == end()) {
    return (*rbegin()).second;
  }

  return lerp((*(itr - 1)).second, (*(itr - 1)).first, (*itr).second,
              (*itr).first, s);
}
}  // namespace TiEV