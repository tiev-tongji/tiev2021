#include <vector>
#include <cmath>
#include "interpolator.h"
using namespace std;
namespace TiEV{

std::vector<std::pair<double, double>> Interpolator::linearInterpolator(
    const std::vector<std::pair<double, double>> &pointlist, double interval){
    vector<pair<double, double>> res;
    res.reserve(pointlist.size());
    if(pointlist.size() == 0) return res;
    res.push_back(pointlist[0]);
    for(int i = 1; i < pointlist.size(); ++i){
        double ax = res.back().first, ay = res.back().second;
        double bx = pointlist[i].first, by = pointlist[i].second;
        double dis = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
        int piece_num;
        for(piece_num = 1; dis / piece_num > interval; ++piece_num);
        double dx = (bx - ax) / piece_num, dy = (by - ay) / piece_num;
        for(int j = 0; j < piece_num - 1; ++j)
            res.push_back(make_pair(ax += dx, ay += dy));
        res.push_back(make_pair(bx, by));
    }
    return res;
}

}