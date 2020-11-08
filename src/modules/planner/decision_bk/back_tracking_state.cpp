
#include "decision_state.h"

namespace TiEV{

/*a demo state implement*/
void BackTracking::entry(){
	cout << "Entry BackTracking State..." << endl;
	vs->print_text("State", "BackTracking", 1);
}

void BackTracking::exit(){
	cout << "Exit BackTracking State..." << endl;
}

void BackTracking::react(PlanningEvent const & e){
    getReferencePath(e.nav_info, false);
	getSafeMap(e.lidar_map, e.lanes);
	vector<pair<double, double>> speed_limit;
	vector<DynamicObj> dynamic_objs;
	bool tracking_obs = false;
	vector<Point> back_reference_path;
	vector<RoadInfo> back_road_infoes;
	for(int i = all_reference_path.size() - 1; i>= 0; i--){
		Point p = all_reference_path[i];
		p.s = fabs(p.s);
		if(all_reference_path[i].s <= 0){
			back_reference_path.push_back(p);
			back_road_infoes.push_back(all_road_infoes[i]);
		}
	}
	for(int i = 0; i < back_reference_path.size(); i++){
		Point point = back_reference_path[i];
		double reference_speed = getSpeedByMode(back_road_infoes[i].speed_mode);
		speed_limit.push_back(make_pair(point.s, reference_speed));
		if(!tracking_obs && !safePoint(point)){
			cout << point.x << " " << point.y << endl;
			tracking_obs = true;
			DynamicObj obj;
			obj.width = 1.77;
			obj.length = 3.9;
			point.v = 0;
			point.t = 0;
			point.angle.setByRad(Angle::PI - point.angle.getRad());
			obj.path.push_back(point);
			dynamic_objs.push_back(obj);
		}
	}
	if(!back_reference_path.empty()){
		reference_path.front().v = fabs(e.nav_info.current_speed);
		cout << "--------------------" << endl;
		cout << "speed:" << reference_path.front().v << endl;
		cout << "--------------------" << endl;
		//dynamic_objs.insert(dynamic_objs.end(), e.dynamic_objs.dynamic_obj_list.begin(), e.dynamic_objs.dynamic_obj_list.end());
		SpeedPath speed_path = SpeedOptimizer::RunSpeedOptimizer(dynamic_objs, back_reference_path, speed_limit, back_reference_path.back().s);
		sv->setSpeedPath(speed_path);
		for(auto& p: speed_path.path) p.v = -p.v;
		updateMaintainedPath(e.nav_info, speed_path.path);
	}

//for visualization
	view_info.nav_info = e.nav_info;
	view_info.lidar_map = e.lidar_map;
	view_info.dynamic_objs = e.dynamic_objs;
	view_info.warning_objs = e.warning_objs;
	view_info.traffic_light = e.traffic_light;
	view_info.lanes = e.lanes;
	view_info.parking_lots = e.parking_lots;
	view_info.all_reference_path = all_reference_path;
	view_info.all_road_infoes = all_road_infoes;
	view_info.reference_path = back_reference_path;
	view_info.road_infoes = back_road_infoes;
	Point car_point;
	car_point.x = 300;
	car_point.y = 75;
	view_info.start_point = car_point;
	dv->setViewInfo(view_info);
	vs->print_text("reference path len", back_reference_path.size());
}

}//end TiEV
