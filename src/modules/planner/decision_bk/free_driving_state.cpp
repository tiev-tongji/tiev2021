#include "decision_state.h"
#include "visualization.h"

namespace TiEV{

/*Free Driving state*/
void FreeDriving::entry() {
	cout << "Entry FreeDriving State..." << endl;
	no_solution.lock(10*1000);
	vs->print_text("State", "FreeDriving", 1);
}

void FreeDriving::exit() {
	cout << "Exit FreeDriving State..." << endl;
}

void FreeDriving::react(PlanningEvent const & e)  {
	if(!e.nav_info.reliable){
        cout << "DEBUG::nav_info" << endl;
        transit<NormalDriving>();
		return;
	}
   
	getInfoByNavinfo(e.nav_info);
	if(completTask()){
		transit<StopState>();
		return;
	}
	if(road_mode == RoadMode::PARKING_MODE){
		transit<Parking>();
		return;
	}else if(road_mode == RoadMode::TRACKING_MODE){
		if(!reference_path.empty()){
			Point p = reference_path.front();
			if(fabs(p.x - 300) < 10 && fabs(p.y - 75) < 10){
				transit<Tracking>();
				return;
			}
		}
	}else if(road_mode == RoadMode::UTURN_MODE){\
		transit<UturnState>();
		return;
	}
	getMap(e.lidar_map, e.lanes);

	for(int i = 0; i < task_points.size(); i++){
		Point task_point = task_points[i];
		if(utils::isInLocalMap(task_point)) target_points.push_back(task_point);
	}
	if(target_points.empty()) target_points = getTargetsOfReference();

	if(plan(e.dynamic_objs, getReferenceSpeed(), true)){
		no_solution.lock(10*1000);
	}
	else speedReplan(e.dynamic_objs);
	updateMaintainedPath(e.nav_info, best_path.path);
//for visualization
	visualization(e);
	//vs->print_text("to GlobalRePlanning", no_solution.isLock());
	/*
	if(!no_solution.isLock() && start_maintained_path.size() < 5){
		transit<GlobalRePlanning>();
		return;
	}
	*/

	if(!best_path.path.empty() && !target_points.empty()){
		bool to_normal_driving = true;
		for(auto point: best_path.path){
			if(point.backward) {
				to_normal_driving = false;
				break;
			}
		}
		if(to_normal_driving && car_speed > getSpeedByMode(3)){
            cout << "DEBUG::path_size" << best_path.path.size() << " " << "target " << target_points.size() << endl;
            transit<NormalDriving>();
        }
	}
}


bool FreeDriving::getMap(const LidarMap& lidar_map, const LaneList& lanes){
    getAbsSafeMap(lidar_map);
	getLaneSafeMap(lanes);
	vs->print_text("to remove lane", max(0, (int)(no_solution.isLock() - 15)));
	if(no_solution.isLock() < 5)
		memset(lane_safe_map, 0x3f, sizeof(lane_safe_map));
	getReachableMap();
	return true;
}
}//end TiEV
