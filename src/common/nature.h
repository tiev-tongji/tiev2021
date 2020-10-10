#ifndef _NATURE_H_
#define _NATURE_H_


/*
This is the nature header of the TiEV autonomouse driving system
*/

#include "nature/unit.h"
#include "nature/timestamp.h"
#include "nature/angle.h"

namespace TiEV{

const int GRID_ROW = 401;
const int GRID_COL = 151;
const float GRID_RESOLUTION = 0.2;
const int CAR_CEN_COL = 75;
const int CAR_CEN_ROW = 300;
const float CAR_WHEEL_BASE = 2.3;
const float CAR_LENGTH = 3.9; //(m)
const float CAR_WIDTH = 2.0; //(m)
const float CAR_FRONT_AXLE_TO_HEAD = 1.02; //(m)
const float CAR_BUMPER_LENGTH = 0.3; //(m)

const float GRAVITY = 9.8;
const float MIU = 0.5;

//calculated by above car info
//const float COLLISION_CIRCLE_BIG_R = 2.19; //(m)
//const float COLLISION_CIRCLE_SMALL_R = 1.0; //(m)
//const float COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE = -1.03; //(m) the car center
//const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 = 0.13; //(m) the first small circle from car head
//const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 = -0.68; //(m) the second small circle from car head
//const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 = -1.48; //(m) the third small circle from car head
//const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 = -2.28; //(m) the fourth small circle from car head

//qzr modified
const float COLLISION_CIRCLE_BIG_R = 2.19; //(m)
const float COLLISION_CIRCLE_SMALL_R = 1.2; //(m)
const float COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE = -1.03; //(m) the car center
const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 = 0.13; //(m) the first small circle from car head
const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 = -0.68; //(m) the second small circle from car head
const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 = -1.48; //(m) the third small circle from car head
const float COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 = -2.28; //(m) the fourth small circle from car head



//calculated by above car info
const float ABS_CAR_WIDTH = 1.77; //(m)
const float ABS_COLLISION_CIRCLE_BIG_R = 2.14; //(m)
const float ABS_COLLISION_CIRCLE_SMALL_R = 0.885; //(m)
const float ABS_COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE = -0.93; //(m) the car center
const float ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 = 0.435; //(m) the first small circle from car head
const float ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 = -0.45; //(m) the second small circle from car head
const float ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 = -1.335; //(m) the third small circle from car head
const float ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 = -2.22; //(m) the fourth small circle from car head
enum LCMCHANNLE
{
	DEFAULT = 0, //for NAVINFO CANINFO
	MAPPING, //for SICKMAP LUXMAP LASERMAP
	PLANNING, // for FUSIONMAP ESROBJINFO OBJECTLIST
	VISION, //for LANE TRAFFICLIGHT STOPLINE
	CONTROL //for REMOTECONTROL SLAMCONTROL ACC AIMPATH AIMPATHINT CANCONTROL
};

//
//
}
#endif // _NATURE_H_
