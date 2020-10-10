#include "decision_state.h"

namespace TiEV{

/*Normal Driving state*/
void FoggyDriving::entry() {
	cout << "Entry FoggyDriving State..." << endl;
	vs->print_text("State", "FoggyDriving", 1);
}

void FoggyDriving::exit() {
	cout << "Exit FoggyDriving State..." << endl;
}

void FoggyDriving::react(PlanningEvent const & e){
	if(!getInfoByNavinfo(e.nav_info)){
		transit<NormalDriving>();
		return;
	}

	for(int i = 0; i < task_points.size(); i++) {
		Point task_point = task_points[i];
		if(utils::isInLocalMap(task_point) && safePoint(task_point) && reachable[(int)task_point.x][(int)task_point.y]) target_points.push_back(task_point);
	}
	if(target_points.empty() && e.lanes.detected) target_points = getTargetsOfLanes(e.lanes);
	if(target_points.empty()) target_points = getTargetsOfReference();
	if(completTask()){
		transit<StopState>();
		return;
	}
	if (road_mode != RoadMode::FOGGY_MODE) {
	    transit<NormalDriving>();
	    return;
	}

	getMap(e.lidar_map, e.lanes);
	for(int i = 0; i < task_points.size(); i++){
		Point task_point = task_points[i];
		if(utils::isInLocalMap(task_point) && safePoint(task_point) && reachable[(int)task_point.x][(int)task_point.y]) target_points.push_back(task_point);
	}
	if(target_points.empty()) target_points = getTargetsOfReference();

	if (!plan(e.dynamic_objs, getReferenceSpeed(), false)) {
	    speedReplan(e.dynamic_objs);
	}
	updateMaintainedPath(e.nav_info, best_path.path);
    visualization(e);
}

bool FoggyDriving::getMap(const LidarMap& lidar_map, const LaneList& lanes){
	LidarMap esr_map;
	memset(&esr_map, 0x00, sizeof(esr_map));
    if(lidar_map.detected){
	    for(int r = 0; r < GRID_ROW; ++r)
	    	for(int c = 0; c < GRID_COL; ++c)
				if((lidar_map.map[r][c] & 0x01) || (lidar_map.map[r][c] & 0x10))
					esr_map.map[r][c] = 0x01;
	}
	esr_map.detected = true;
	getAbsSafeMap(esr_map);
	getLaneSafeMap(lanes);
	getReachableMap();
	return true;
}

}//end TiEV
