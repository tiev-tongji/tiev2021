#include "decision_state.h"
#include "visualization.h"

namespace TiEV{

/*Free Driving state*/
void UturnState::entry() {
	cout << "Entry UturnState State..." << endl;
	no_target.lock(5*1000);
	vs->print_text("State", "UturnState", 1);
}

void UturnState::exit() {
	cout << "Exit UturnState State..." << endl;
}

void UturnState::react(PlanningEvent const & e)  {
	if(!e.nav_info.reliable){
		transit<NormalDriving>();
		return;
	}

	getInfoByNavinfo(e.nav_info);

	if(completTask()){
		transit<StopState>();
		return;
	}
	if(road_mode != RoadMode::UTURN_MODE){
		transit<NormalDriving>();
		return;
	}

	getMap(e.lidar_map, e.lanes);

	for(int i = 0; i < task_points.size(); i++){
		Point task_point = task_points[i];
		if(utils::isInLocalMap(task_point) && reachable[(int)task_point.x][(int)task_point.y]) target_points.push_back(task_point);
	}
	if(target_points.empty()){
		target_points = getTargetsOfReference();
		/*
		if(getTargetOfUturn(e.nav_info)) target_points.push_back(uturn_target);
		else if(!no_target.isLock()){
			transit<GlobalPlanning>();
			return;
		}
		*/
	}
	if(!target_points.empty()){
		no_target.lock(5*1000);
		/*
		double dis_to_refer_path = 0x3f3f3f3f;
		for(int i = 0; i < reference_path.size(); i++){
			double dis = hypot(fabs(reference_path[i].x - CAR_CEN_ROW), fabs(reference_path[i].y - CAR_CEN_COL));
			if(dis < dis_to_refer_path) dis_to_refer_path = dis;
		}
		if(dis_to_refer_path < 1.2 / GRID_RESOLUTION && fabs(target_points.front().angle.getDegree()) < 45){
			transit<NormalDriving>();
			return;
		}
		*/
	}

	bool plan_success = plan(e.dynamic_objs, getReferenceSpeed(), true);

	if(!plan_success) speedReplan(e.dynamic_objs);

	updateMaintainedPath(e.nav_info, best_path.path);

	visualization(e);
}

bool UturnState::getInfoByNavinfo(const NavInfo& nav_info){
	car_speed = nav_info.current_speed;
	if(!nav_info.reliable) return false;
	updateTaskPoints(nav_info);
	getReferencePath(nav_info, true);
	road_manager->getMaintainedPath(nav_info, maintained_path);
	getStartMaintainedPath(nav_info);
	if(!road_infoes.empty()){
		road_mode = road_infoes.front().mode;
		speed_mode = road_infoes.front().speed_mode;
	}
	return true;
}

bool UturnState::getMap(const LidarMap& lidar_map, const LaneList& lanes){
	getAbsSafeMap(lidar_map);
	getReachableMap();
	return true;
}

bool UturnState::getTargetOfUturn(const NavInfo& nav_info){
	if(uturn_target.x < 0){
		for(int i = reference_path.size() - 1; i >= 0; i--){
			Point p = reference_path[i];
			if(p.x < 330){
				if(absSafePoint(p) && reachable[(int)p.x][(int)p.y]){
					uturn_target = reference_path[i];
					return true;
				}
			}
		}
		for(int i = 1; i < reference_path.size(); i++){
			Point p = reference_path[i];
			if(absSafePoint(p) && reachable[(int)p.x][(int)p.y]){
				uturn_target = reference_path[i];
				return true;
			}
		}
		return false;
	}else{
		uturn_target.updateLocalCoordinate(nav_info.current_position);
		if(!absSafePoint(uturn_target) || !reachable[(int)uturn_target.x][(int)uturn_target.y]){
			uturn_target.x = -1;
			return false;
		}else return true;
	}
}

}//end TiEV
