#include "decision_event.h"

namespace TiEV{

void PlanningEvent::refresh(){
//message
	msg_manager->getNavInfo(nav_info);
	msg_manager->getSlamInfo(slam_info);
	msg_manager->getMap(lidar_map);
	msg_manager->getDynamicObjList(dynamic_objs);
	msg_manager->getWarningObjList(warning_objs);
	msg_manager->getTrafficLight(traffic_light);
	msg_manager->getLaneList(lanes);
	msg_manager->getParkingLotList(parking_lots);
	msg_manager->getPedestrian(pedestrian);

	vs->print_text("lat", nav_info.current_position.lat, 1, false);
	vs->print_text("lon", nav_info.current_position.lon, 1, false);
	vs->print_text("heading", nav_info.current_position.heading.getRad(), 1, false);
	vs->print_text("SLAM x", slam_info.x, 1, false);
	vs->print_text("SLAM y", slam_info.y, 1, false);
	vs->print_text("SLAM heading", slam_info.heading, 1, false);
	vs->print_text("SLAM status", slam_info.reliable, 1, false);
	vs->print_text("speed(m/s)", nav_info.current_speed, 1);
	vs->print_text("speed(km/h)", nav_info.current_speed*3.6, 1);
	vs->print_text("RTK", nav_info.reliable, 1);

	if(traffic_light.detected){
		vs->print_text("traffic light", "detected", 2);
		vs->print_text("traffic light <=", traffic_light.left, 2);
		vs->print_text("traffic light ||", traffic_light.straight, 2);
		vs->print_text("traffic light =>", traffic_light.right, 2);
	}
	else{
		vs->print_text("traffic light", "none", 2);
		vs->print_text("traffic light <=", "none", 2);
		vs->print_text("traffic light ||", "none", 2);
		vs->print_text("traffic light =>", "none", 2);
	}
	if(lanes.detected) vs->print_text("lane line", "detected", 2);
	else vs->print_text("lane line", "none", 2);
	if(parking_lots.detected) vs->print_text("parking lot", "detected", 2);
	else vs->print_text("parking lot", "none", 2);
	if(pedestrian.detected) vs->print_text("pedestrian", "detected", 2);
	else vs->print_text("pedestrian", "none", 2);
}


}//namespace TiEV 
