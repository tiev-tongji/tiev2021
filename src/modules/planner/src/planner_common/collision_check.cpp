#include "collision_check.h"
#include <math.h>

namespace TiEV {
using namespace std;

bool collision(double x, double y, double ang, const double dis_map[MAX_ROW][MAX_COL], double expansion_r) {
    double sin_ang = sin(ang), cos_ang = cos(ang);
    int circle_center_2_x = round(x + cos_ang * (circle_dis2 / GRID_RESOLUTION));
    int circle_center_2_y = round(y + sin_ang * (circle_dis2 / GRID_RESOLUTION));
    double big_r = (expansion_r + COLLISION_CIRCLE_BIG_R) / GRID_RESOLUTION;
    double small_r = (expansion_r + COLLISION_CIRCLE_SMALL_R) / GRID_RESOLUTION;
    if (circle_center_2_x >= 0 && circle_center_2_x < MAX_ROW &&
        circle_center_2_y >= 0 && circle_center_2_y < MAX_COL) {
        if(dis_map[circle_center_2_x][circle_center_2_y] > big_r) return false;
        if(dis_map[circle_center_2_x][circle_center_2_y] <= small_r) return true;
    }
    int circle_center_1_x = round(x + cos_ang * (circle_dis1 / GRID_RESOLUTION));
    int circle_center_1_y = round(y + sin_ang * (circle_dis1 / GRID_RESOLUTION));
    if (circle_center_1_x >= 0 && circle_center_1_x < MAX_ROW &&
        circle_center_1_y >= 0 && circle_center_1_y < MAX_COL) {
        if (dis_map[circle_center_1_x][circle_center_1_y] <= small_r) return true;
    }
    int circle_center_3_x = round(x + cos_ang * (circle_dis3 / GRID_RESOLUTION));
    int circle_center_3_y = round(y + sin_ang * (circle_dis3 / GRID_RESOLUTION));
    if (circle_center_3_x >= 0 && circle_center_3_x < MAX_ROW &&
        circle_center_3_y >= 0 && circle_center_3_y < MAX_COL) {
        if(dis_map[circle_center_3_x][circle_center_3_y] <= small_r) return true;
    }

    return false;
}

bool collision(const Pose& pose, const double dis_map[MAX_ROW][MAX_COL], double expansion_r) {
    return collision(pose.x, pose.y, pose.ang, dis_map, expansion_r);
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