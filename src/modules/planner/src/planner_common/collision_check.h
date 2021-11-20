#ifndef __COLLISION_CHECK__H__
#define __COLLISION_CHECK__H__
#include <vector>

#include "const.h"
#include "pose.h"
namespace TiEV {
/**
 * 这是途灵的碰撞检测，总共4个圆形包裹途灵
 * 首先检测大圆，再检测3个小圆
 * 坐标系定义：行为横轴，列为纵轴，当前车辆朝向为pi
 */
constexpr float CAR_WHEEL_BASE = 2.3;
constexpr float CAR_LENGTH     = 3.9;  //(m)
// constexpr float CAR_WIDTH              = 1.15;  //(m)
constexpr float CAR_WIDTH              = 1.2;   //(m)
constexpr float CAR_FRONT_AXLE_TO_HEAD = 1.02;  //(m)
constexpr float CAR_BUMPER_LENGTH      = 0.3;   //(m)

constexpr double COLLISION_CIRCLE_BIG_R   = 2.3;   // m
constexpr double COLLISION_CIRCLE_SMALL_R = 1.25;  // m

constexpr double circle_dis2 = -((CAR_LENGTH + CAR_BUMPER_LENGTH) / 2 -
                                 (CAR_FRONT_AXLE_TO_HEAD + CAR_BUMPER_LENGTH));
constexpr double circle_dis1 = circle_dis2 + COLLISION_CIRCLE_SMALL_R;
constexpr double circle_dis3 = circle_dis2 - COLLISION_CIRCLE_SMALL_R;

bool collision(double x, double y, double ang,
               const double dis_map[MAX_ROW][MAX_COL],
               double       expansion_r = 0.0);
bool collision(const Pose& pose, const double dis_map[MAX_ROW][MAX_COL],
               double expansion_r = 0);
bool collision(const std::vector<Pose>& path,
               const double dis_map[MAX_ROW][MAX_COL], double expansion_r = 0);
bool collision(double x, double y, const double dis_map[MAX_ROW][MAX_COL],
               double expansion_r = 0.0);

}  // namespace TiEV

#endif  //!__COLLISION_CHECK__H__