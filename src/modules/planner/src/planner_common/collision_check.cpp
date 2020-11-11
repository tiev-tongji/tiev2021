#include "collision_check.h"
#include <math.h>

namespace TiEV {
using namespace std;
bool collision(const Pose& pose, const double dis_map[MAX_ROW][MAX_COL], double expansion_r) {
    Point2d current_point(pose.x, pose.y);
    Point2d direct_vect(cos(pose.ang), sin(pose.ang));
    Point2d circle_center2 = current_point + direct_vect * (circle_dis2 / GRID_RESOLUTION);
    if(!circle_center2.in_map()) return true;
    if(dis_map[int(circle_center2.x)][int(circle_center2.y)] > (expansion_r + COLLISION_CIRCLE_BIG_R) / GRID_RESOLUTION) return false;
    if(dis_map[int(circle_center2.x)][int(circle_center2.y)] <= (expansion_r + COLLISION_CIRCLE_SMALL_R) / GRID_RESOLUTION) return true;
    Point2d circle_center1 = current_point + direct_vect * (circle_dis1 / GRID_RESOLUTION);
    if(!circle_center1.in_map()) return true;
    if(dis_map[int(circle_center1.x)][int(circle_center1.y)] <= (expansion_r + COLLISION_CIRCLE_SMALL_R) / GRID_RESOLUTION) return true;
    Point2d circle_center3 = current_point + direct_vect * (circle_dis3 / GRID_RESOLUTION);
    if(!circle_center3.in_map()) return true;
    if(dis_map[int(circle_center3.x)][int(circle_center3.y)] <= (expansion_r + COLLISION_CIRCLE_SMALL_R) / GRID_RESOLUTION) return true;
    return false;
}

bool collision(const vector<Pose>& path, const double dis_map[MAX_ROW][MAX_COL], double expansion_r) {
    double check_interval = 2.0;  // m
    double next_check_s   = 0;
    for(const Pose& p : path) {
        if(p.s < next_check_s) continue;
        next_check_s = p.s + check_interval;
        if(collision(p, dis_map, expansion_r)) return true;
    }
    return false;
}
}  // namespace TiEV