#ifndef _DECISION_EVENT_H
#define _DECISION_EVENT_H

#include "tinyfsm.hpp"
#include "message_manager.h"
#include "visualization.h"

namespace TiEV{

using namespace tinyfsm;
class PlanningEvent : public Event{
	public:
	/*from message manager*/
		NavInfo nav_info;				//Nav info
		SlamInfo slam_info;		//Slam Location
		LidarMap lidar_map;				//Lidar map
		DynamicObjList dynamic_objs;	//Dynamic objects
		WarningObjList warning_objs;	//Warning objects
		TrafficLight traffic_light;		//Traffic light
		LaneList lanes;					//Lane line
		ParkingLotList parking_lots;	//Parking lots
		Pedestrian pedestrian;			//Pedestrian

	public:
		PlanningEvent(){
			msg_manager = MessageManager::getInstance();
			vs = Visualization::getInstance();
		}
		void refresh();
	private:
		MessageManager *msg_manager;
		Visualization *vs;
};

}//namespace TiEV end

#endif
