#include "decision_state.h"
#include "visualization.h"
#include <unistd.h>

namespace TiEV{

/*Free Driving state*/
void Parking::entry() {
	cout << "Entry Parking State..." << endl;
	vs->print_text("State", "Parking", 1);
	slam_control.mapping = 1;
}

void Parking::exit() {
	cout << "Exit Parking State..." << endl;
}

void Parking::react(PlanningEvent const & e)  {
	MessageManager::getInstance()->publishSlamControl(slam_control);
//path
	ParkingLotList parking_lots;

	NavInfo fake_nav_info;
	fake_nav_info.detected = fake_nav_info.reliable = true;
	fake_nav_info.current_speed = e.nav_info.current_speed;

	if(e.slam_info.reliable){
		fake_nav_info.current_position = Point::fromUTMXY(e.slam_info.x, e.slam_info.y);
		fake_nav_info.current_position.heading.setByRad(e.slam_info.heading);
		fake_nav_info.current_position.x = CAR_CEN_ROW;
		fake_nav_info.current_position.y = CAR_CEN_COL;
		fake_nav_info.current_position.angle.setByRad(0);
		fake_nav_info.reliable = true;
	}else if(e.nav_info.reliable) fake_nav_info = e.nav_info;
	else{
		transit<MapFree>();
		return;
	}

	getInfoByNavinfo(fake_nav_info);
	if(e.parking_lots.detected) road_manager->maintainParkingLotList(fake_nav_info, e.parking_lots);
	road_manager->getMaintainedParkingLotList(fake_nav_info, parking_lots);

//for planner
	getMap(e.lidar_map, e.lanes);
//Path_Planner path_planner;
	vector<Point> target_points;
	if(parking_lots.detected) target_points = getTargetsOfParkingLots(parking_lots);
    else target_points = getTargetsOfParkingLots(e.parking_lots);
	bool target_from_referpath = false;
	if(target_points.empty()){
		target_points = getTargetsOfReference();
		target_from_referpath = true;
	}else target_points.resize(1);

    for(int i = 0; i < target_points.size(); i++){
        if(fabs(target_points[i].x - CAR_CEN_ROW) < 3
			&& fabs(target_points[i].y - CAR_CEN_COL < 2)
			&& fabs(target_points[i].angle.getDegree()) < 15){
            transit<StopState>();
            return;
        }
    }

    bool replanning = false;
    for(auto p: maintained_path){
        if(!safePoint(p)){
            replanning = true;
            break;
        }
    }
    if(maintained_path.empty()) replanning = true;

//path planning
	if(replanning || target_from_referpath){
		path_planner->setAbsSafeMap(abs_safe_map);
		path_planner->setLaneSafeMap(lane_safe_map);
		path_planner->setStartMaintainedPath(start_maintained_path);
		path_planner->setTargets(target_points);
		path_planner->setBackwardEnabled(true);
		path_planner->setCurrentSpeed(fake_nav_info.current_speed);
		path_planner->setDynamicObjList(e.dynamic_objs);
		path_planner->setVelocityLimit(5);
		path_planner->plan();
		path_planner->getResults(planned_paths);
		bool have_best_path = getBestPath();
		vs->print_text("best path", have_best_path);
		vs->print_text("best path size", (int)best_path.path.size());

		if(!best_path.path.empty()){
			sv->setSpeedPath(best_path);
			updateMaintainedPath(fake_nav_info, best_path.path);
		}else{
			if(!fake_nav_info.reliable) updateMaintainedPath(fake_nav_info, best_path.path);
			sv->setSpeedPath(best_path);
		}
	}else{
	    vector<Point> tmp_speed_path;
		vector<DynamicObj> dynamic_objs;
		dynamic_objs.insert(dynamic_objs.end(), e.dynamic_objs.dynamic_obj_list.begin(), e.dynamic_objs.dynamic_obj_list.end());
	    int break_index = 0;
		if(!maintained_path.empty()){
			tmp_speed_path.push_back(maintained_path.front());
			break_index = 1;
		}
	    for(int i = 1; i < maintained_path.size(); i++){
	        Point p = maintained_path[i];
	        if(p.backward != maintained_path[i-1].backward){
	            break_index = i;
	            break;
	         }
	        tmp_speed_path.push_back(p);
	    }
	    vector<pair<double, double>> speed_limit;
	    for(auto p: tmp_speed_path) speed_limit.push_back(make_pair(p.s, 2));
	    SpeedPath speed_path;
	    if(!tmp_speed_path.empty()){
	        tmp_speed_path.front().v = fake_nav_info.current_speed;
	        speed_limit.back().second = 0;
		    speed_path = SpeedOptimizer::RunSpeedOptimizer(dynamic_objs, tmp_speed_path, speed_limit, tmp_speed_path.back().s);
	        for(auto& p: speed_path.path) if(p.backward) p.v = -p.v;
		    sv->setSpeedPath(speed_path);
	    }
	    speed_path.path.insert(speed_path.path.end(), maintained_path.begin()+break_index, maintained_path.end());
		vs->print_text("updatepath", speed_path.path.size());
		updateMaintainedPath(fake_nav_info, speed_path.path);
	}

//for visualization
	view_info.nav_info = fake_nav_info;
	view_info.lidar_map = e.lidar_map;
	view_info.dynamic_objs = e.dynamic_objs;
	view_info.warning_objs = e.warning_objs;
	view_info.traffic_light = e.traffic_light;
	view_info.lanes = e.lanes;
	if(parking_lots.detected) view_info.parking_lots = parking_lots;
	else view_info.parking_lots = e.parking_lots;

	view_info.targets = target_points;
	view_info.all_reference_path = all_reference_path;
	view_info.all_road_infoes = all_road_infoes;
	view_info.reference_path = reference_path;
	view_info.road_infoes = road_infoes;
	view_info.best_path = maintained_path;
	Point car_point;
	car_point.x = 300;
	car_point.y = 75;
	if(start_maintained_path.empty()) view_info.start_point = car_point;
	else view_info.start_point = start_maintained_path.back();
	dv->setViewInfo(view_info);
	vs->print_text("reference path len", reference_path.size());
	vs->print_text("targets number", target_points.size());
}

}//end TiEV
