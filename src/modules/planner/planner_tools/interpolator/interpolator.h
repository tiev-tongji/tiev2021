#ifndef _INTERPOLATOR_H_
#define _INTERPOLATOR_H_

#include <vector>
using namespace std;

namespace TiEV {

class Interpolator{
public:
    std::vector<std::pair<double, double>> linearInterpolator(
        const std::vector<std::pair<double, double>> &pointlist,
        double interval = 1);
};

}

#endif
