#include "decision_state.h"

namespace TiEV{

/*Normal Driving state*/
void NormalDriving::entry() {
	cout << "Entry NormalDriving State..." << endl;
	no_solution.lock(15*1000);
	vs->print_text("State", "NormalDriving", 1);

}

void NormalDriving::exit() {
	cout << "Exit NormalDriving State..." << endl;
}

void NormalDriving::react(PlanningEvent const & e)  {
	if(!e.nav_info.reliable){
		if(e.lanes.detected){
			transit<LaneDriving>();
			return;
		}else{
			transit<MapFree>();
			return;
		}
	}

	getInfoByNavinfo(e.nav_info);
	if(completTask()){
		transit<StopState>();
		return;
	}
	getMap(e.lidar_map, e.lanes);
	switch(road_mode){
		case RoadMode::INTERSECTION:
			if(!not_in_lane){
				transit<Intersection>();
				return;
			}else break;
		case RoadMode::FOGGY_MODE: 
			transit<FoggyDriving>();
			return;
		case RoadMode::UTURN_MODE:
			transit<UturnState>();
			return;
		case RoadMode::PARKING_MODE:
			transit<Parking>();
			return;
		case RoadMode::TRACKING_MODE:
			if(!reference_path.empty()){
				Point p = reference_path.front();
				if(fabs(p.x - 300) < 10 && fabs(p.y - 75) < 10){
					transit<Tracking>();
					return;
				}
			}
	}

	for(int i = 0; i < task_points.size(); i++){
		Point task_point = task_points[i];
		if(utils::isInLocalMap(task_point)) target_points.push_back(task_point);
	}
	if(target_points.empty()) target_points = getTargetsOfReference();
	if(!target_points.empty())

	if(!plan(e.dynamic_objs, getReferenceSpeed())) speedReplan(e.dynamic_objs);
    else no_solution.lock(15*1000);
	updateMaintainedPath(e.nav_info, best_path.path);
//for visualization
	visualization(e);
	vs->print_text("to Freedriving", no_solution.isLock());
	if(!no_solution.isLock()){
		transit<FreeDriving>();
		return;
	}
}

}//end TiEV
