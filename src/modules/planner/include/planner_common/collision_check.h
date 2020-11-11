#ifndef __COLLISION_CHECK__H__
#define __COLLISION_CHECK__H__
#include "const.h"
#include "pose.h"
#include <vector>
namespace TiEV {
/**
 * 这是途灵的碰撞检测，总共4个圆形包裹途灵
 * 首先检测大圆，再检测3个小圆
 * 坐标系定义：行为横轴，列为纵轴，当前车辆朝向为-pi
 */
const float CAR_WHEEL_BASE         = 2.3;
const float CAR_LENGTH             = 3.9;   //(m)
const float CAR_WIDTH              = 2.0;   //(m)
const float CAR_FRONT_AXLE_TO_HEAD = 1.02;  //(m)
const float CAR_BUMPER_LENGTH      = 0.3;   //(m)

const double COLLISION_CIRCLE_BIG_R   = 2.3;   // m
const double COLLISION_CIRCLE_SMALL_R = 1.15;  // m

const double circle_dis2 = -((CAR_LENGTH + CAR_BUMPER_LENGTH) / 2 - (CAR_FRONT_AXLE_TO_HEAD + CAR_BUMPER_LENGTH));
const double circle_dis1 = circle_dis2 + COLLISION_CIRCLE_SMALL_R;
const double circle_dis3 = circle_dis2 - COLLISION_CIRCLE_SMALL_R;

bool collision(const Pose& pose, const double dis_map[MAX_ROW][MAX_COL], double expansion_r = 0);
bool collision(const std::vector<Pose>& path, const double dis_map[MAX_ROW][MAX_COL], double expansion_r = 0);

}  // namespace TiEV

#endif  //!__COLLISION_CHECK__H__