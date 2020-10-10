#include "decision_state.h"

namespace TiEV{

/*Intersection state*/
void Intersection::entry() {
	cout << "Entry Intersection State..." << endl;
	intersection_lock.lock(30*1000);
	vs->print_text("State", "Intersection", 1);
}

void Intersection::exit() {
	cout << "Exit Intersection State..." << endl;
}

void Intersection::react(PlanningEvent const & e)  {
	if(!intersection_lock.isLock()){
		transit<FreeDriving>();
		return;
	}
	if(!e.nav_info.reliable){
			transit<NormalDriving>();
			return;
	}
    getInfoByNavinfo(e.nav_info);

	if(road_mode != RoadMode::INTERSECTION){
		transit<NormalDriving>();
		return;
	}
	getMap(e.lidar_map, e.lanes);

	target_points = getTargetsOfReference();

	bool send_stop_line = false;
	DynamicObjList tmp_dyn_list;
	if(e.dynamic_objs.detected) tmp_dyn_list = e.dynamic_objs;
	if((!road_infoes.empty()) && e.traffic_light.detected){
		if(!((road_infoes.front().direction == RoadDirection::STRAIGHT && e.traffic_light.straight)
			|| (road_infoes.front().direction == RoadDirection::LEFT && e.traffic_light.left)
			|| (road_infoes.front().direction == RoadDirection::RIGHT && e.traffic_light.right)))
			send_stop_line = true;
	}
	vs->print_text("T_send_stop_line", send_stop_line, 2);

	cout << "Debug:send_stop_line:" << send_stop_line << endl;
	if(send_stop_line){
		if(e.lanes.detected && e.lanes.lane_list[e.lanes.current_id].stop_point.y >= 0){
			DynamicObj obj;
			obj.width = 8;
			obj.length = 0.5;
			obj.heading = 0.0;
			obj.type = ObjectType::UNKNOWN;
			obj.path.push_back(e.lanes.lane_list[e.lanes.current_id].stop_point);
			tmp_dyn_list.dynamic_obj_list.push_back(obj);
		}
		else{
			int stop_line_index = -1;
			for(int i = 0; i < road_infoes.size(); i++){
				if(road_infoes[i].event == EventPoint::STOP_LINE){
					stop_line_index = i;
					break;
				}
			}
			if(stop_line_index >= 0){
				for(int l = 1; l <= road_infoes[stop_line_index].lane_num; l++){
					DynamicObj obj;
					double stop_point_lateral_dis = (l - road_infoes[stop_line_index].lane_seq) * \
					road_infoes[stop_line_index].lane_width;
					Point stop_point = utils::getLateralPoint(reference_path[stop_line_index], stop_point_lateral_dis);
					cout << "reference path angle:" << reference_path[stop_line_index].angle.getRad() << " " << reference_path[stop_line_index].angle.getDegree() << endl;
					cout << "stop_point angle:" << stop_point.angle.getRad() << endl;
					stop_point.s = 0;
					stop_point.v = 0;
					stop_point.k = 0;
					stop_point.t = 0;
					stop_point.a = 0;
					obj.width = 8;
					obj.length = 0.5;
					obj.heading = stop_point.angle.getRad();
					obj.type = ObjectType::UNKNOWN;
					obj.path.push_back(stop_point);
					tmp_dyn_list.dynamic_obj_list.push_back(obj);
					vs->print_text("t_obj", stop_point.x, 2);
				}
			}
		}
	}

	tmp_dyn_list.detected = !tmp_dyn_list.dynamic_obj_list.empty();

	//replan is needed regardless of the result of planner
	//the result above only considers history map
	if(plan(tmp_dyn_list, getReferenceSpeed()))
		maintained_path = best_path.path;

	speedReplan(tmp_dyn_list);
	updateMaintainedPath(e.nav_info, best_path.path);
	visualization(e);
}

bool Intersection::getMap(const LidarMap& lidar_map, const LaneList& lanes){
	LidarMap hist_map;
	memset(&hist_map, 0x00, sizeof(hist_map));
    if(lidar_map.detected){
	    for(int i = 0; i < GRID_ROW; ++i)
	    	for(int j = 0; j < GRID_COL; ++j)
	    		hist_map.map[i][j] = (lidar_map.map[i][j] & 0x01);
     }
	 hist_map.detected = true;
	 getAbsSafeMap(lidar_map);
	 getLaneSafeMap(lanes);
	 getReachableMap();
	 return true;
}

}//end TiEV
