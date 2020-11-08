#include "tinyfsm.hpp"
#include "decision_fsm.h"
#include <iostream>
#include <typeinfo>
using namespace std;
using namespace tinyfsm;

namespace TiEV{
//TiEV begin


void DecisionFSM::react(PlanningEvent const & e){
	cout << "void react(PlanningEvent) was not implemented in State: " << typeid(*current_state_ptr).name() << endl;
}

void DecisionFSM::refresh(){
//clear
	all_reference_path.clear();
	all_road_infoes.clear();
	reference_path.clear();
	road_infoes.clear();
	maintained_path.clear();
	best_path.path.clear();
	planned_paths.clear();
    start_maintained_path.clear();
	task_points.clear();
	target_points.clear();
	view_info.clear();
	block_lane.clear();
	block_lane.resize(21, false);

	car_speed = 0;
	road_mode = -1;
	speed_mode = -1;
	offset_left = 0;
	offset_right = 0;
	car_current_hd_map_lane_id  = 0;
	car_dis_to_refer = 0; 
	memset(abs_safe_map, 0x3f, sizeof(abs_safe_map));
	memset(lane_safe_map, 0x3f, sizeof(lane_safe_map));
	if(!Config::getInstance()->tasks.empty()){
		task_points = Config::getInstance()->tasks.front().task_points;
	}
}

bool DecisionFSM::getInfoByNavinfo(const NavInfo& nav_info){
	car_speed = nav_info.current_speed;
	if(!nav_info.reliable) return false;
	updateTaskPoints(nav_info);
	getReferencePath(nav_info);
	road_manager->getMaintainedPath(nav_info, maintained_path);
	getStartMaintainedPath(nav_info);
	if(!road_infoes.empty()){
		road_mode = road_infoes.front().mode;
		vs->print_text("RoadMode", road_mode);
		speed_mode = road_infoes.front().speed_mode;
		switch(speed_mode){
			case 0: vs->print_text("SpeedMode","Back");break;
			case 1: vs->print_text("SpeedMode","Stop");break;
			case 2: vs->print_text("SpeedMode","VeryLow");break;
			case 3: vs->print_text("SpeedMode","Low");break;
			case 4: vs->print_text("SpeedMode","Mid");break;
			case 5: vs->print_text("SpeedMode","High");break;
			case 6: vs->print_text("SpeedMode","VeryHigh");break;
		}
	}
	return true;
}

bool DecisionFSM::getMap(const LidarMap& lidar_map, const LaneList& lanes){
	getLaneSafeMap(lanes);
    getAbsSafeMap(lidar_map);
	getReachableMap();
	return true;
}

bool DecisionFSM::plan(const DynamicObjList &dynamic_objs, double max_speed, bool reverse){
	DynamicObjList obj_list;
	for(auto point: start_maintained_path){
		if(!absSafePoint(point)){
			DynamicObj obj;
			obj.width = 1.77;
			obj.length = 3.9;
			point.v = 0;
			point.t = 0;
			point.angle.setByRad(Angle::PI - point.angle.getRad());
			obj.path.push_back(point);
			obj_list.dynamic_obj_list.push_back(obj);
			obj_list.detected = true;
			break;
		}
	}
    if(dynamic_objs.detected){
		for(DynamicObj ob: dynamic_objs.dynamic_obj_list){
			if(ob.type == ObjectType::PEDESTRIAN && !reference_path.empty()){
				double ob_to_refer = 0x3f3f3f3f;
				int ob_index_in_refer_path = -1;
				int count = -1;
				for(auto path_point: reference_path){
					count++;
					double dis = fabs(path_point.x - ob.path.front().x) + fabs(path_point.y - ob.path.front().y);
					if(dis < ob_to_refer){
						ob_to_refer = dis;
						ob_index_in_refer_path = count;
					}
				}
				if(ob_index_in_refer_path >= 0) ob_to_refer = utils::getLateralDistance(reference_path[ob_index_in_refer_path], ob.path.front());
				Point car_point;
				car_point.x = CAR_CEN_ROW;
				car_point.y = CAR_CEN_COL;
				double car_to_refer = utils::getLateralDistance(reference_path.front(), car_point);
				double car_to_right_line = car_to_refer - ((1 - road_infoes.front().lane_seq) * road_infoes.front().lane_width - road_infoes.front().lane_width/2);
				double ob_car_dis = ob_to_refer - car_to_refer;
				if(ob_car_dis > road_infoes.front().lane_width || ob_car_dis < -car_to_right_line) continue;
				obj_list.dynamic_obj_list.push_back(ob);
			}else obj_list.dynamic_obj_list.push_back(ob);
		}
		obj_list.detected = true;
	}
	path_planner->setAbsSafeMap(abs_safe_map);
	path_planner->setLaneSafeMap(lane_safe_map);
	path_planner->setStartMaintainedPath(start_maintained_path);
	path_planner->setTargets(target_points);
	path_planner->setBackwardEnabled(reverse);
	path_planner->setCurrentSpeed(car_speed);
	path_planner->setDynamicObjList(obj_list);
	path_planner->setVelocityLimit(max_speed);
	path_planner->plan();
	path_planner->getResults(planned_paths);
	getBestPath();
	sv->setSpeedPath(best_path);
	return !best_path.path.empty();
}

void DecisionFSM::speedReplan(const DynamicObjList &dynamic_obj_list){
	vector<Point> tmp_speed_path;
	vector<DynamicObj> dynamic_objs;
	dynamic_objs.insert(dynamic_objs.end(), dynamic_obj_list.dynamic_obj_list.begin(), dynamic_obj_list.dynamic_obj_list.end());
	int break_index = 0;
	if(!maintained_path.empty()){
		tmp_speed_path.push_back(maintained_path.front());
		break_index = 1;
	}
	for(int i = 1; i < maintained_path.size(); i++){
	    Point p = maintained_path[i];
	    if(p.backward != maintained_path[i-1].backward){
	        break;
	     }
	    break_index = i + 1;
	    tmp_speed_path.push_back(p);
	}
	for(auto point: tmp_speed_path){
		if(!absSafePoint(point)){
			DynamicObj obj;
			obj.width = 1.77;
			obj.length = 3.9;
			point.v = 0;
			point.t = 0;
			point.angle.setByRad(Angle::PI - point.angle.getRad());
			obj.path.push_back(point);
			dynamic_objs.push_back(obj);
			break;
		}
	}
	vector<pair<double, double>> speed_limit;
	for(auto p: tmp_speed_path) speed_limit.push_back(make_pair(p.s, getReferenceSpeed()));
	SpeedPath speed_path;
	if(!tmp_speed_path.empty()){
	    tmp_speed_path.front().v = car_speed;
	    speed_limit.back().second = 0;
	    speed_path = SpeedOptimizer::RunSpeedOptimizer(dynamic_objs, tmp_speed_path, speed_limit, tmp_speed_path.back().s);
	    for(auto& p: speed_path.path) if(p.backward) p.v = -p.v;
	    sv->setSpeedPath(speed_path);
	}
	if(!maintained_path.empty())
	    speed_path.path.insert(speed_path.path.end(), maintained_path.begin()+break_index, maintained_path.end());
	best_path = speed_path;
}

void DecisionFSM::visualization(const PlanningEvent& e){
	view_info.nav_info = e.nav_info;
	view_info.lidar_map = e.lidar_map;
	view_info.dynamic_objs = e.dynamic_objs;
	view_info.warning_objs = e.warning_objs;
	view_info.traffic_light = e.traffic_light;
	view_info.lanes = e.lanes;
	view_info.parking_lots = e.parking_lots;

	view_info.targets = target_points;
	view_info.all_reference_path = all_reference_path;
	view_info.all_road_infoes = all_road_infoes;
	view_info.reference_path = reference_path;
	view_info.road_infoes = road_infoes;
	view_info.best_path = best_path.path;
	Point car_point;
	car_point.x = CAR_CEN_ROW; 
	car_point.y = CAR_CEN_COL;
	if(start_maintained_path.empty())	view_info.start_point = car_point;
	else view_info.start_point = start_maintained_path.back();
	dv->setViewInfo(view_info);
	vs->print_text("reference path len", reference_path.size());
	vs->print_text("targets number", target_points.size());
}

vector<Point> DecisionFSM::getTargetsOfParkingLots(ParkingLotList const &parking_lots){
	vector<Point> targets;
	if(!parking_lots.detected) return targets;
	for(int i = 0; i < parking_lots.parking_lot_list.size(); i++){
		Point target;
		target.v = 0;
		ParkingLot parking_lot = parking_lots.parking_lot_list[i];
		target.x = (parking_lot.left_back.x + parking_lot.left_front.x + \
		parking_lot.right_back.x + parking_lot.right_front.x) / 4;
		target.y = (parking_lot.left_back.y + parking_lot.left_front.y + \
		parking_lot.right_back.y + parking_lot.right_front.y) / 4;
		double heading = utils::getAngleOfAlongDirection(parking_lot.left_back, parking_lot.left_front);
		target.angle.setByRad(heading); 
		Point parking_point = utils::getLongitudePoint(target, CAR_LENGTH/2 - CAR_FRONT_AXLE_TO_HEAD);
		if(safePoint(parking_point)) targets.push_back(parking_point);
	}
	return targets;
}

vector<Point> DecisionFSM::getTargetsOfLanes(LaneList const &lanes){
	vector<Point> targets;
	if(!lanes.detected) return targets;
	for(int i = 0; i < lanes.lane_list.size(); i++){
		Lane lane = lanes.lane_list[i];
		Point target;
		target.v = getReferenceSpeed();
		target.x = (lane.left_line.points.back().x + \
		lane.right_line.points.back().x) / 2;
		target.y = (lane.left_line.points.back().y + \
		lane.right_line.points.back().y) / 2;
		double heading = utils::getAngleOfNormalDirection(lane.right_line.points.back(), \
		lane.left_line.points.back());
		target.angle.setByRad(heading);
		if(reachable[(int)target.x][(int)target.y] && safePoint(target)) targets.push_back(target);
	}
	return targets;
}

void DecisionFSM::updateTaskPoints(const NavInfo& nav_info){
	if(task_points.empty()) return;
	for(auto& target: task_points){
		target.updateLocalCoordinate(nav_info.current_position);
	}
}

vector<Point> DecisionFSM::getTargetsByMap(){
	vector<Point> targets;
	vector<Point> path;
	for(int i = 0; i < 50; i++){
		Point p;
		p.x = CAR_CEN_ROW - i;
		p.y = CAR_CEN_COL;
		p.angle.setByRad(0);
		p.s = i * GRID_RESOLUTION;
		p.v = 2;
		path.push_back(p);
	}
	for(int i = path.size() - 1; i >= 3; i--){
		Point target = path[i];
		if(safePoint(target) && reachable[(int)target.x][(int)target.y]){
			targets.push_back(target);
			break;
		}else{
			Point right_target = utils::getLateralPoint(target, -3);
			if(safePoint(right_target) && reachable[(int)right_target.x][(int)right_target.y]){
				targets.push_back(right_target);
				break;
			}
			Point left_target = utils::getLateralPoint(target, 3);
			if(safePoint(left_target) && reachable[(int)left_target.x][(int)left_target.y]){
				targets.push_back(left_target);
				break;
			}
		}
	}
	return targets;
}

vector<Point> DecisionFSM::getTargetsOfReference(){
	vector<Point> targets;
	vector<bool> target_in_lane;
	target_in_lane.resize(21, false);
	bool have_lane = false;
	for(int i = reference_path.size() - 1; i >= 20; i--){
		if(road_infoes[i].lane_num > 0){
			have_lane = true;
			break;
		}
	}
	for(int  i = reference_path.size() - 1; i >= 10;){
		double target_speed = getSpeedByMode(road_infoes[i].speed_mode);
		Point &target = reference_path[i];
		target.v = target_speed;
		if(road_infoes[i].lane_num != 0){
			have_lane = true;
			for(int l = 1; l <= road_infoes[i].lane_num; l++){
				if(target_in_lane[l] || block_lane[l]) continue;
				double lateral_dis = (l - road_infoes[i].lane_seq) * road_infoes[i].lane_width;
				if(l < car_current_hd_map_lane_id) lateral_dis += offset_right;
				else if(l > car_current_hd_map_lane_id) lateral_dis += offset_left;
				else lateral_dis += (offset_left + offset_right) / 2;
				Point lane_point = utils::getLateralPoint(target, lateral_dis);
				if(safePoint(lane_point) && reachable[(int)lane_point.x][(int)lane_point.y]){
					lane_point.v = target_speed;
					targets.push_back(lane_point);
					target_in_lane[l] = true;
				}
				if(targets.size() >= road_infoes[i].lane_num) return targets;
			}
		}else if(!have_lane){
			for(int l = 1; l <= 3; l++){
				if(target_in_lane[l]) continue;
				double lateral_dis = (l - 2) * LATERAL_INTERVAL;
				Point lane_point = utils::getLateralPoint(target, lateral_dis);
				if(safePoint(lane_point) && reachable[(int)lane_point.x][(int)lane_point.y]){
					lane_point.v = target_speed;
					targets.push_back(lane_point);
					target_in_lane[l] = true;
				}
			}
		}
		for(int j = i; j >= 0; j--, i--){
			if(target.s - reference_path[j].s >= LONGITUDE_INTERVAL){
				break;
			}
		}
	}
	return targets;
}

bool DecisionFSM::getReferencePath(NavInfo const &nav_info, bool opposite){
	if(!nav_info.reliable) return false;
	road_manager->getLocalReferencePath(nav_info, all_reference_path, all_road_infoes, opposite);
	for(int i = 0; i < all_reference_path.size(); i++){
        if(all_reference_path[i].s < 0) continue;
        reference_path.push_back(all_reference_path[i]);
        road_infoes.push_back(all_road_infoes[i]);
    }
    if(reference_path.empty()) return false;
    return true;
}

double DecisionFSM::getReferenceSpeed(){
	if(road_mode == RoadMode::INTERSECTION) speed_mode = 4;
	return getSpeedByMode(speed_mode);
}

double DecisionFSM::getSpeedByMode(int _speed_mode){
	switch(_speed_mode){
		case 0 : return -1;
		case 1 : return 0;
		case 2 : return 2.7;
		case 3 : return 5;
		case 4 : return 7;
		case 5 : return 11.1;
		case 6 : return 16.6;
		default : return 0;
	}
}

void DecisionFSM::getAbsSafeMap(LidarMap const &lidar_map){
    const int dx[] = {0, 0, -1, 1};
    const int dy[] = {-1, 1, 0, 0};

    memset(abs_safe_map, 0x3f, sizeof(abs_safe_map));
	queue<pair<int, int>> obj_que;

	for(int r = 0; r < GRID_ROW; r++){
		for(int c = 0; c < GRID_COL; c++){
			if(lidar_map.map[r][c] != 0 && lidar_map.map[r][c] != 0x4){
				abs_safe_map[r][c] = 0;
				obj_que.push(make_pair(r, c));
			}
		}
	}

    while(!obj_que.empty()){
		const int x = obj_que.front().first;
		const int y = obj_que.front().second;
		obj_que.pop();
        for(int i = 0; i < 4; ++i){
            const int tx = x + dx[i], ty = y + dy[i];
            if(tx >= 0 && tx < GRID_ROW && ty >= 0 && ty < GRID_COL \
			&& abs_safe_map[tx][ty] > abs_safe_map[x][y] + 1){
				abs_safe_map[tx][ty] = abs_safe_map[x][y] + 1;
				obj_que.push(make_pair(tx, ty));
            }
        }
	}
}

void DecisionFSM::getLaneSafeMap(const LaneList& lanes){
    const int dx[] = {0, 0, -1, 1};
    const int dy[] = {-1, 1, 0, 0};

    memset(lane_safe_map, 0x3f, sizeof(lane_safe_map));
	queue<pair<int, int>> obj_que;

	if(!all_reference_path.empty() && !reference_path.empty()){//have hd_map
		//calculate car dis to reference_path and index of lane of car
		if(!reference_path.empty()){
			Point p = reference_path.front();
			double theta = Angle::PI - p.angle.getRad();
			utils::normalizeAngle(theta);
			double k = tan(theta);
			Point car_point;
			car_point.x = CAR_CEN_ROW;
			car_point.y = CAR_CEN_COL;
			car_point.angle.setByRad(0);
			car_dis_to_refer = utils::getLateralDistance(p, car_point);
			if(road_infoes.front().lane_num > 0){
				double lane_width_ = road_infoes.front().lane_width;
				int lane_id = road_infoes.front().lane_seq;
				int lane_num = road_infoes.front().lane_num;
				car_current_hd_map_lane_id = lane_id + floor((car_dis_to_refer + lane_width_/2) / lane_width_);
			}
			vs->print_text("dis_to_refer(m)", car_dis_to_refer);
			vs->print_text("current_lane_id", car_current_hd_map_lane_id);

			//calculate the lateral offset by lanes from visual
			if(lanes.detected && car_current_hd_map_lane_id <= road_infoes.front().lane_num && car_current_hd_map_lane_id > 0){
				double current_left_line_distance = lanes.lane_list[lanes.current_id].left_line.distance;
				double current_right_line_distance = lanes.lane_list[lanes.current_id].right_line.distance;
				LineType left_line_type = lanes.lane_list[lanes.current_id].left_line.type;
				LineType right_line_type = lanes.lane_list[lanes.current_id].right_line.type;
				int matching_lane_id = car_current_hd_map_lane_id;
				double current_hd_left_line_distance = (car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width + road_infoes.front().lane_width/2 - car_dis_to_refer;
				double current_hd_right_line_distance = (car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width - road_infoes.front().lane_width/2 - car_dis_to_refer;
				cout << "visual left:" << current_left_line_distance << " visual right:" << current_right_line_distance << endl;
				cout << "hdmap left:" << current_hd_left_line_distance << " hdmap right:" << current_hd_right_line_distance << endl;
				double visual_distance[] = {current_left_line_distance, current_right_line_distance};
				double hdmap_distance[] = {current_hd_left_line_distance, current_hd_right_line_distance};
				double dmax = 1e13;
				int ans = -1;
				for(int ii = 0; ii < 2; ii++) {
					for(int jj = 0; jj < 2; jj++) {
						double delta = fabs(visual_distance[ii] - hdmap_distance[jj]);
						if(delta < dmax) {
							dmax = delta;
							ans = ii * 2 + jj;
						}
					}
				}
				if(ans == 0) {
					offset_left = visual_distance[0] - hdmap_distance[0];
					offset_right = visual_distance[1] - hdmap_distance[1];
				} else if (ans == 1) {
					offset_left = visual_distance[0] - hdmap_distance[1];
					offset_right = offset_left;
				} else if (ans == 2) {
					offset_right= visual_distance[1] - hdmap_distance[0];
					offset_left = offset_right;
				} else if (ans == 3) {
					offset_left = visual_distance[0] - hdmap_distance[0];
					offset_right = visual_distance[1] - hdmap_distance[1];
				}

				/*
				if(current_left_line_distance < current_right_line_distance){
					if(current_hd_left_line_distance < current_hd_right_line_distance){
						offset_left = current_left_line_distance - current_hd_left_line_distance;
						offset_right = current_hd_right_line_distance - current_right_line_distance;
					}else{
						offset_right = current_hd_right_line_distance + current_left_line_distance;
						offset_left = offset_right;
					}
				}else{
					if(current_hd_left_line_distance > current_hd_right_line_distance){
						offset_right = current_hd_right_line_distance - current_right_line_distance;
						offset_left = current_left_line_distance - current_hd_left_line_distance;
					}else{
						offset_left = -(current_right_line_distance + current_hd_left_line_distance);
						offset_right = offset_left;
					}
				}
				*/
				/*
				offset_left = fabs(lanes.lane_list[lanes.current_id].left_line.distance) - ((car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width + road_infoes.front().lane_width/2 - car_dis_to_refer);
				offset_right = -(((car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width - road_infoes.front().lane_width/2 - car_dis_to_refer) + lanes.lane_list[lanes.current_id].right_line.distance);
				*/
			}
			if(road_infoes.front().lane_num > 0){
				double lane_width = road_infoes.front().lane_width;
				int lane_id = road_infoes.front().lane_seq;
				int lane_num = road_infoes.front().lane_num;
				double left_line_boder = (lane_num - lane_id) * lane_width + lane_width/2 + offset_left;
				double right_line_boder = (1-lane_id) * lane_width - lane_width/2 + offset_right;
				if(car_dis_to_refer <= right_line_boder || car_dis_to_refer >= left_line_boder) not_in_lane = true;
			}
		}
		vs->print_text("lane_offset_left(m)", offset_left);
		vs->print_text("lane_offset_right(m)", offset_right);

		//add line to obstacle
		for(int i = 0; i < all_road_infoes.size(); i++){
			double lane_width = all_road_infoes[i].lane_width;
			int lane_id = all_road_infoes[i].lane_seq;
			int lane_num = all_road_infoes[i].lane_num;
			BlockType block_type = all_road_infoes[i].block_type;
			RoadDirection road_direction = all_road_infoes[i].direction;

			if(0 == lane_num) continue;
			
			//add the border line
			Point boder_left, boder_right;
			double line_lateral_dis_left = 0, line_lateral_dis_right = 0;
			if(block_type == BlockType::BlockLeft){
				line_lateral_dis_left = (lane_num - lane_id) * lane_width + lane_width/2 + offset_left;
				boder_left = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_left);
			}else if(block_type == BlockType::BlockRight){
				line_lateral_dis_right = (1-lane_id) * lane_width - lane_width/2 + offset_right;
				boder_right = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_right);
			}else if(block_type == BlockType::BlockAll){
				line_lateral_dis_left = (lane_num - lane_id) * lane_width + lane_width/2 + offset_left;
				line_lateral_dis_right = (1-lane_id) * lane_width - lane_width/2 + offset_right;
				boder_left = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_left);
				boder_right = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_right);
			}
			if((car_dis_to_refer <= line_lateral_dis_right || car_dis_to_refer >= line_lateral_dis_left) && all_reference_path[i].s < 10) continue;
				
			if(utils::isInLocalMap(boder_left)){
				lane_safe_map[(int)boder_left.x][(int)boder_left.y] = 0;
				obj_que.push(make_pair((int)boder_left.x, (int)boder_left.y));
			}
			if(utils::isInLocalMap(boder_right)){
				lane_safe_map[(int)boder_right.x][(int)boder_right.y] = 0;
				obj_que.push(make_pair((int)boder_right.x, (int)boder_right.y));
			}
			
			if(RoadMode::INTERSECTION == all_road_infoes[i].mode){
				//add solid line in intersection obs
				for(int l = 1; l < lane_num; l++){
					Point p;
					if(l < car_current_hd_map_lane_id){ 
						double line_lateral_dis = (l-lane_id) * lane_width + lane_width/2 + offset_right;
						p = utils::getLateralPoint(all_reference_path[i], line_lateral_dis);
					}
					else{
						double line_lateral_dis = (l-lane_id) * lane_width + lane_width/2 + offset_left;
						p = utils::getLateralPoint(all_reference_path[i], line_lateral_dis);
					}
					if(utils::isInLocalMap(p)) {
						lane_safe_map[(int)p.x][(int)p.y] = 0;
						obj_que.push(make_pair((int)p.x, (int)p.y));
					}
				}

				//block the increct lane
				if(road_mode == RoadMode::INTERSECTION) continue;
				vector<int> lane_direction(21, 0x07);
				if(lane_num == 2){
					lane_direction[1] = RoadDirection::RIGHT | RoadDirection::STRAIGHT;
					lane_direction[2] = RoadDirection::LEFT | RoadDirection::STRAIGHT;
				}else if(lane_num > 2){
					for(int l = 1; l <= lane_num; l++){
						if(l == 1) lane_direction[l] = RoadDirection::RIGHT;
						else if(l == lane_num) lane_direction[l] = RoadDirection::LEFT;
						else{
							lane_direction[l] = RoadDirection::STRAIGHT;
						}
					}
				}
				if(lanes.detected){
					for(int l = 0; l < lanes.lane_list.size(); l++){
						int vision_lane_id_in_hdmap = l - lanes.current_id + car_current_hd_map_lane_id;
						if(vision_lane_id_in_hdmap < 1 || vision_lane_id_in_hdmap > lane_num || lanes.lane_list[l].type == 0x00) continue;
						lane_direction[vision_lane_id_in_hdmap] = lanes.lane_list[l].type;
					}
				}
				for(int l = 1; l <= lane_num; l++){
					if(all_reference_path[i].s < 0) continue;
                    cout << "now direction:" << road_direction << " lane" << l << " direction:" << lane_direction[l] << endl;
					if(road_direction & lane_direction[l]) continue;
					block_lane[l] = true;
					for(double intervel = 0; intervel < lane_width; intervel += 0.5){
						double op_lateral_dis;
						if(l <= car_current_hd_map_lane_id) op_lateral_dis = (l-lane_id) * lane_width - lane_width/2 + offset_right + intervel;
						else op_lateral_dis = (l-lane_id) * lane_width - lane_width/2 + offset_left + intervel;
						Point op = utils::getLateralPoint(all_reference_path[i], op_lateral_dis);
						if(utils::isInLocalMap(op)) {
							lane_safe_map[(int)op.x][(int)op.y] = 0;
							obj_que.push(make_pair((int)op.x, (int)op.y));
						}
					}
				}
			}
		}
	}else if(lanes.detected){//no hd_map
		for(int i = 0; i < lanes.lane_list.size(); i++){
			Lane lane = lanes.lane_list[i];
			if(lane.left_line.type == LineType::SOLID){
				LaneLine line = lane.left_line;
				for(int j = 0; j < line.points.size(); j++){
					int x = line.points[j].x;
					int y = line.points[j].y;
					if(x >= 0 && y >= 0 && x < GRID_ROW && y < GRID_COL){
						lane_safe_map[x][y] = 0;
						obj_que.push(make_pair(x, y));
					}
				}
			}
			if(lane.right_line.type == SOLID){
				LaneLine line = lane.right_line;
				for(int j = 0; j < line.points.size(); j++){
					int x = line.points[j].x;
					int y = line.points[j].y;
					if(x >= 0 && y >= 0 && x < GRID_ROW && y < GRID_COL){
						lane_safe_map[x][y] = 0;
						obj_que.push(make_pair(x, y));
					}
				}
			}
		}
	}

    while(!obj_que.empty()){
		const int x = obj_que.front().first;
		const int y = obj_que.front().second;
		obj_que.pop();
        for(int i = 0; i < 4; ++i){
            const int tx = x + dx[i], ty = y + dy[i];
            if(tx >= 0 && tx < GRID_ROW && ty >= 0 && ty < GRID_COL \
			&& lane_safe_map[tx][ty] > lane_safe_map[x][y] + 1){
				lane_safe_map[tx][ty] = lane_safe_map[x][y] + 1;
				obj_que.push(make_pair(tx, ty));
            }
        }
	}

}

void DecisionFSM::getSafeMap(LidarMap const &lidar_map, LaneList const &lanes){
    const int dx[] = {0, 0, -1, 1};
    const int dy[] = {-1, 1, 0, 0};

    memset(safe_map, 0x3f, sizeof(safe_map));
	queue<pair<int, int>> obj_que;

//TODO remove dynamic objects
	for(int r = 0; r < GRID_ROW; r++){
		for(int c = 0; c < GRID_COL; c++){
			if(lidar_map.map[r][c] != 0 && !(0x04 & lidar_map.map[r][c])){
				safe_map[r][c] = 0;
				obj_que.push(make_pair(r, c));
			}
		}
	}

	if(!all_reference_path.empty() && !reference_path.empty()){//have hd_map
		//calculate car dis to reference_path and index of lane of car
		if(!reference_path.empty()){
			Point p = reference_path.front();
			double theta = Angle::PI - p.angle.getRad();
			utils::normalizeAngle(theta);
			double k = tan(theta);
			Point car_point;
			car_point.x = CAR_CEN_ROW;
			car_point.y = CAR_CEN_COL;
			car_point.angle.setByRad(0);
			car_dis_to_refer = utils::getLateralDistance(p, car_point);
			if(road_infoes.front().lane_num > 0){
				double lane_width_ = road_infoes.front().lane_width;
				int lane_id = road_infoes.front().lane_seq;
				int lane_num = road_infoes.front().lane_num;
				car_current_hd_map_lane_id = lane_id + floor((car_dis_to_refer + lane_width_/2) / lane_width_);
			}
			vs->print_text("dis_to_refer(m)", car_dis_to_refer);
			vs->print_text("current_lane_id", car_current_hd_map_lane_id);

			//calculate the lateral offset by lanes from visual
			if(lanes.detected && car_current_hd_map_lane_id <= road_infoes.front().lane_num && car_current_hd_map_lane_id > 0){
				double current_left_line_distance = lanes.lane_list[lanes.current_id].left_line.distance;
				double current_right_line_distance = lanes.lane_list[lanes.current_id].right_line.distance;
				LineType left_line_type = lanes.lane_list[lanes.current_id].left_line.type;
				LineType right_line_type = lanes.lane_list[lanes.current_id].right_line.type;
				int matching_lane_id = car_current_hd_map_lane_id;
				double current_hd_left_line_distance = (car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width + road_infoes.front().lane_width/2 - car_dis_to_refer;
				double current_hd_right_line_distance = (car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width - road_infoes.front().lane_width/2 - car_dis_to_refer;
				cout << "visual left:" << current_left_line_distance << " visual right:" << current_right_line_distance << endl;
				cout << "hdmap left:" << current_hd_left_line_distance << " hdmap right:" << current_hd_right_line_distance << endl;
				double visual_distance[] = {current_left_line_distance, current_right_line_distance};
				double hdmap_distance[] = {current_hd_left_line_distance, current_hd_right_line_distance};
				double dmax = 1e13;
				int ans = -1;
				for(int ii = 0; ii < 2; ii++) {
					for(int jj = 0; jj < 2; jj++) {
						double delta = fabs(visual_distance[ii] - hdmap_distance[jj]);
						if(delta < dmax) {
							dmax = delta;
							ans = ii * 2 + jj;
						}
					}
				}
				if(ans == 0) {
					offset_left = visual_distance[0] - hdmap_distance[0];
					offset_right = visual_distance[1] - hdmap_distance[1];
				} else if (ans == 1) {
					offset_left = visual_distance[0] - hdmap_distance[1];
					offset_right = offset_left;
				} else if (ans == 2) {
					offset_right= visual_distance[1] - hdmap_distance[0];
					offset_left = offset_right;
				} else if (ans == 3) {
					offset_left = visual_distance[0] - hdmap_distance[0];
					offset_right = visual_distance[1] - hdmap_distance[1];
				}

				/*
				if(current_left_line_distance < current_right_line_distance){
					if(current_hd_left_line_distance < current_hd_right_line_distance){
						offset_left = current_left_line_distance - current_hd_left_line_distance;
						offset_right = current_hd_right_line_distance - current_right_line_distance;
					}else{
						offset_right = current_hd_right_line_distance + current_left_line_distance;
						offset_left = offset_right;
					}
				}else{
					if(current_hd_left_line_distance > current_hd_right_line_distance){
						offset_right = current_hd_right_line_distance - current_right_line_distance;
						offset_left = current_left_line_distance - current_hd_left_line_distance;
					}else{
						offset_left = -(current_right_line_distance + current_hd_left_line_distance);
						offset_right = offset_left;
					}
				}
				*/
				/*
				offset_left = fabs(lanes.lane_list[lanes.current_id].left_line.distance) - ((car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width + road_infoes.front().lane_width/2 - car_dis_to_refer);
				offset_right = -(((car_current_hd_map_lane_id - road_infoes.front().lane_seq) * road_infoes.front().lane_width - road_infoes.front().lane_width/2 - car_dis_to_refer) + lanes.lane_list[lanes.current_id].right_line.distance);
				*/
			}
			if(road_infoes.front().lane_num > 0){
				double lane_width = road_infoes.front().lane_width;
				int lane_id = road_infoes.front().lane_seq;
				int lane_num = road_infoes.front().lane_num;
				double left_line_boder = (lane_num - lane_id) * lane_width + lane_width/2 + offset_left;
				double right_line_boder = (1-lane_id) * lane_width - lane_width/2 + offset_right;
				if(car_dis_to_refer <= right_line_boder || car_dis_to_refer >= left_line_boder) not_in_lane = true;
			}
		}
		vs->print_text("lane_offset_left(m)", offset_left);
		vs->print_text("lane_offset_right(m)", offset_right);

		//add line to obstacle
		for(int i = 0; i < all_road_infoes.size(); i++){
			double lane_width = all_road_infoes[i].lane_width;
			int lane_id = all_road_infoes[i].lane_seq;
			int lane_num = all_road_infoes[i].lane_num;
			BlockType block_type = all_road_infoes[i].block_type;
			RoadDirection road_direction = all_road_infoes[i].direction;

			if(0 == lane_num) continue;
			
			//add the border line
			Point boder_left, boder_right;
			double line_lateral_dis_left = 0, line_lateral_dis_right = 0;
			if(block_type == BlockType::BlockLeft){
				line_lateral_dis_left = (lane_num - lane_id) * lane_width + lane_width/2 + offset_left;
				boder_left = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_left);
			}else if(block_type == BlockType::BlockRight){
				line_lateral_dis_right = (1-lane_id) * lane_width - lane_width/2 + offset_right;
				boder_right = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_right);
			}else if(block_type == BlockType::BlockAll){
				line_lateral_dis_left = (lane_num - lane_id) * lane_width + lane_width/2 + offset_left;
				line_lateral_dis_right = (1-lane_id) * lane_width - lane_width/2 + offset_right;
				boder_left = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_left);
				boder_right = utils::getLateralPoint(all_reference_path[i], line_lateral_dis_right);
			}
			if((car_dis_to_refer <= line_lateral_dis_right || car_dis_to_refer >= line_lateral_dis_left) && all_reference_path[i].s < 10) continue;
				
			if(utils::isInLocalMap(boder_left)){
				safe_map[(int)boder_left.x][(int)boder_left.y] = 0;
				obj_que.push(make_pair((int)boder_left.x, (int)boder_left.y));
			}
			if(utils::isInLocalMap(boder_right)){
				safe_map[(int)boder_right.x][(int)boder_right.y] = 0;
				obj_que.push(make_pair((int)boder_right.x, (int)boder_right.y));
			}
			
			if(RoadMode::INTERSECTION == all_road_infoes[i].mode){
				//add solid line in intersection obs
				for(int l = 1; l < lane_num; l++){
					Point p;
					if(l < car_current_hd_map_lane_id){ 
						double line_lateral_dis = (l-lane_id) * lane_width + lane_width/2 + offset_right;
						p = utils::getLateralPoint(all_reference_path[i], line_lateral_dis);
					}
					else{
						double line_lateral_dis = (l-lane_id) * lane_width + lane_width/2 + offset_left;
						p = utils::getLateralPoint(all_reference_path[i], line_lateral_dis);
					}
					if(utils::isInLocalMap(p)) {
						safe_map[(int)p.x][(int)p.y] = 0;
						obj_que.push(make_pair((int)p.x, (int)p.y));
					}
				}

				//block the increct lane
				if(road_mode == RoadMode::INTERSECTION) continue;
				vector<int> lane_direction(21, 0x07);
				if(lane_num == 2){
					lane_direction[1] = RoadDirection::RIGHT | RoadDirection::STRAIGHT;
					lane_direction[2] = RoadDirection::LEFT | RoadDirection::STRAIGHT;
				}else if(lane_num > 2){
					for(int l = 1; l <= lane_num; l++){
						if(l == 1) lane_direction[l] = RoadDirection::RIGHT;
						else if(l == lane_num) lane_direction[l] = RoadDirection::LEFT;
						else{
							lane_direction[l] = RoadDirection::STRAIGHT;
						}
					}
				}
				if(lanes.detected){
					for(int l = 0; l < lanes.lane_list.size(); l++){
						int vision_lane_id_in_hdmap = l - lanes.current_id + car_current_hd_map_lane_id;
						if(vision_lane_id_in_hdmap < 1 || vision_lane_id_in_hdmap > lane_num || lanes.lane_list[l].type == 0x00) continue;
						lane_direction[vision_lane_id_in_hdmap] = lanes.lane_list[l].type;
					}
				}
				for(int l = 1; l <= lane_num; l++){
					if(all_reference_path[i].s < 0) continue;
                    cout << "now direction:" << road_direction << " lane" << l << " direction:" << lane_direction[l] << endl;
					if(road_direction & lane_direction[l]) continue;
					block_lane[l] = true;
					for(double intervel = 0; intervel < lane_width; intervel += 0.5){
						double op_lateral_dis;
						if(l <= car_current_hd_map_lane_id) op_lateral_dis = (l-lane_id) * lane_width - lane_width/2 + offset_right + intervel;
						else op_lateral_dis = (l-lane_id) * lane_width - lane_width/2 + offset_left + intervel;
						Point op = utils::getLateralPoint(all_reference_path[i], op_lateral_dis);
						if(utils::isInLocalMap(op)) {
							safe_map[(int)op.x][(int)op.y] = 0;
							obj_que.push(make_pair((int)op.x, (int)op.y));
						}
					}
				}
			}
		}
	}else if(lanes.detected){//no hd_map
		for(int i = 0; i < lanes.lane_list.size(); i++){
			Lane lane = lanes.lane_list[i];
			if(lane.left_line.type == LineType::SOLID){
				LaneLine line = lane.left_line;
				for(int j = 0; j < line.points.size(); j++){
					int x = line.points[j].x;
					int y = line.points[j].y;
					if(x >= 0 && y >= 0 && x < GRID_ROW && y < GRID_COL){
						safe_map[x][y] = 0;
						obj_que.push(make_pair(x, y));
					}
				}
			}
			if(lane.right_line.type == SOLID){
				LaneLine line = lane.right_line;
				for(int j = 0; j < line.points.size(); j++){
					int x = line.points[j].x;
					int y = line.points[j].y;
					if(x >= 0 && y >= 0 && x < GRID_ROW && y < GRID_COL){
						safe_map[x][y] = 0;
						obj_que.push(make_pair(x, y));
					}
				}
			}
		}
	}

    while(!obj_que.empty()){
		const int x = obj_que.front().first;
		const int y = obj_que.front().second;
		obj_que.pop();
        for(int i = 0; i < 4; ++i){
            const int tx = x + dx[i], ty = y + dy[i];
            if(tx >= 0 && tx < GRID_ROW && ty >= 0 && ty < GRID_COL \
			&& safe_map[tx][ty] > safe_map[x][y] + 1){
				safe_map[tx][ty] = safe_map[x][y] + 1;
				obj_que.push(make_pair(tx, ty));
            }
        }
	}
}

void DecisionFSM::getReachableMap(){
    const int dx[] = {0, 0, -1, 1};
    const int dy[] = {-1, 1, 0, 0};

    memset(reachable, false, sizeof(reachable));
	queue<pair<int, int>> obj_que;
	obj_que.push(make_pair(CAR_CEN_ROW, CAR_CEN_COL));

    while(!obj_que.empty()){
		const int x = obj_que.front().first;
		const int y = obj_que.front().second;
		obj_que.pop();
        for(int i = 0; i < 4; ++i){
            const int tx = x + dx[i], ty = y + dy[i];
            if(tx >= 0 && tx < GRID_ROW && ty >= 0 && ty < GRID_COL \
			&& reachable[tx][ty] == false && lane_safe_map[tx][ty] > ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION && abs_safe_map[tx][ty] > COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION){
				reachable[tx][ty] = true;
				obj_que.push(make_pair(tx, ty));
            }
        }
	}
}

void DecisionFSM::getStartMaintainedPath(NavInfo const &nav_info){
	start_maintained_path.clear();
	if(maintained_path.empty()) return;
	else{
		double maintained_s =(nav_info.current_speed * nav_info.current_speed) / (2 * MIU * GRAVITY);
		double car_far_dis = hypot(fabs(maintained_path.front().x - CAR_CEN_ROW),fabs(maintained_path.front().y - CAR_CEN_COL)) * GRID_RESOLUTION;
		vs->print_text("maintained dis(m)", maintained_s);
		vs->print_text("car deviation(m)", car_far_dis);
		if( car_far_dis > Config::getInstance()->car_away_limit_meter) return;
		for(int i = 0; i < maintained_path.size(); i++){
			start_maintained_path.push_back(maintained_path[i]);
			if(maintained_s < maintained_path[i].s) break;
		}
	}
}

bool DecisionFSM::getBestPath(){
	int best_path_id = -1;
	double best_path_cost = 0x3f3f3f3f;
	int best_lane_id = -1;
	for(int i = 0; i < planned_paths.size(); i++){
		double average_speed = 0, average_curve = 0, distance_near_obs = 0x3f3f3f3f, max_curve = 0, distance_of_last_target = 0;
		vector<Point> &path = planned_paths[i].path;
		int n = 0;
		for(int j = 0; j < path.size(); j++){
			int r = path[j].x;
			int c = path[j].y;
			if(path[j].v == 0) continue;
			//cout << "Debug: " << r << " " << c << " " << path[j].s << " " << path[j].v << " " << path[j].a << " " << path[j].k << endl;
			n++;
			average_speed += fabs(path[j].v);
			average_curve += fabs(path[j].k);
			if(r < 0 || c < 0 || r >= GRID_ROW || c >= GRID_COL) continue;
			if(abs_safe_map[r][c] < distance_near_obs) distance_near_obs = abs_safe_map[r][c];
			if(max_curve < fabs(path[j].k)) max_curve = fabs(path[j].k);
		}
		average_speed /= (n + 1e-8); 
		average_curve /= (n + 1e-8);
		Point target_p = path.back();
		double t_min_dis = 0x3f3f3f3f;
		int t_target_in_refer_id = -1;
		int t_target_lane_id = -1;
		double t_target_dis_to_refer = 0;
		for(int i = 0; i < reference_path.size(); i++){
			Point path_point = reference_path[i];
			double dis = fabs(path_point.x - last_target_point.x) + fabs(path_point.y - last_target_point.y);
			if(dis > t_min_dis) continue;
			t_min_dis = dis;
			t_target_in_refer_id = i;
		}
		if(t_target_in_refer_id >= 0){
			t_target_dis_to_refer = utils::getLateralDistance(reference_path[t_target_in_refer_id], target_p);
			t_target_lane_id = road_infoes[t_target_in_refer_id].lane_seq + \
			floor((t_target_dis_to_refer + road_infoes[t_target_in_refer_id].lane_width/2) / road_infoes[t_target_in_refer_id].lane_width);
		}else{
			t_target_dis_to_refer = 0;
			t_target_lane_id = 0;
		}
		double target_lane_cost = 0;
		cout << "Debug: ";
		cout << "last_target_lane_id:" << last_target_lane_id << " last_target_dis_to_refer:" << last_target_dis_to_refer << endl;
		cout << "Debug: ";
		cout << "target_lane_id:" << t_target_lane_id << " target_dis_to_refer:" << t_target_dis_to_refer << endl;
		if(last_target_lane_id > 0 && t_target_lane_id > 0 && road_infoes[t_target_lane_id].lane_num == last_target_lane_num) target_lane_cost = fabs(t_target_lane_id - last_target_lane_id) * 100;
		else target_lane_cost = fabs(last_target_dis_to_refer - t_target_dis_to_refer) * 30;
		double path_cost = 500 / (average_speed + 0.1) + 100 * average_curve + 30 * max_curve + 50 / (distance_near_obs + 1e-8) + target_lane_cost;
		cout << "Debug: ";
		cout << "path index:" << i << " path cost:" << path_cost << " n:" << n << " average_speed:" << average_speed << " average_curve:" << average_curve << " max_curve" << max_curve << " distance_near_obs:" << distance_near_obs << " target_lane_cost:" << target_lane_cost << endl;
		if(path_cost < best_path_cost){
			best_path_id = i;
			best_path_cost = path_cost;
			best_lane_id = t_target_lane_id;
		} 
	}
	/*
	if(best_lane_id != last_target_lane_id) {
		cout << "Debug:Changelane" << endl;
	}
	*/
	if(best_path_id < 0) return false;
	else{
		best_path = planned_paths[best_path_id];
		last_target_point = best_path.path.back();
		double min_dis = 0x3f3f3f3f;
		int target_in_refer_id = -1;
		for(int i = 0; i < reference_path.size(); i++){
			Point path_point = reference_path[i];
			double dis = fabs(path_point.x - last_target_point.x) + fabs(path_point.y - last_target_point.y);
			if(dis > min_dis) continue;
			min_dis = dis;
			target_in_refer_id = i;
		}
		if(target_in_refer_id >= 0){
			last_target_dis_to_refer = utils::getLateralDistance(reference_path[target_in_refer_id], last_target_point);
			last_target_lane_id = road_infoes[target_in_refer_id].lane_seq + \
			floor((last_target_dis_to_refer + road_infoes[last_target_lane_id].lane_width/2) / road_infoes[last_target_lane_id].lane_width);
			last_target_lane_num = road_infoes[target_in_refer_id].lane_num;
		}else{
			last_target_dis_to_refer = 0;
			last_target_lane_id = 0;
		}
	}
	return true;
}

void DecisionFSM::updateMaintainedPath(NavInfo const &nav_info, vector<Point> const &path){
	road_manager->maintainPath(nav_info, path);
}

bool DecisionFSM::safePoint(Point const &point){
	double expanded_r = (car_speed * 0.06) / GRID_RESOLUTION;
	if(point.x < 0 || point.x >= GRID_ROW || point.y < 0 || point.y >= GRID_COL) return false;
	double cos_ang = point.angle.cos();
	double sin_ang = point.angle.sin();
	double big_circle_x = point.x - COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE * cos_ang / GRID_RESOLUTION;
	double big_circle_y = point.y + COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] > expanded_r + ABS_COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION \
	&& lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] > ABS_COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION) return true;
	double small_circle_x_1 = point.x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_1 = point.y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_1)][(int)round(small_circle_y_1)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
	|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	double small_circle_x_2 = point.x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_2 = point.y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_2)][(int)round(small_circle_y_2)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
	|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	double small_circle_x_3 = point.x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_3 = point.y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_3)][(int)round(small_circle_y_3)] <= expanded_r + COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
	|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	double small_circle_x_4 = point.x - COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_4 = point.y + COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_4)][(int)round(small_circle_y_4)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION \
	|| lane_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] <= ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	return true;
}

bool DecisionFSM::absSafePoint(Point const &point){
	double expanded_r = (car_speed * 0.06) / GRID_RESOLUTION;
	if(point.x < 0 || point.x >= GRID_ROW || point.y < 0 || point.y >= GRID_COL) return false;
	double cos_ang = point.angle.cos();
	double sin_ang = point.angle.sin();
	double big_circle_x = point.x - ABS_COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE * cos_ang / GRID_RESOLUTION;
	double big_circle_y = point.y + ABS_COLLISION_CIRCLE_BIG_L_TO_CAR_FRONT_AXLE * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(big_circle_x)][(int)round(big_circle_y)] > expanded_r + ABS_COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION) return true;
	double small_circle_x_1 = point.x - ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_1 = point.y + ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_1 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_1)][(int)round(small_circle_y_1)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	double small_circle_x_2 = point.x - ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_2 = point.y + ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_2 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_2)][(int)round(small_circle_y_2)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	double small_circle_x_3 = point.x - ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_3 = point.y + ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_3 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_3)][(int)round(small_circle_y_3)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	double small_circle_x_4 = point.x - ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 * cos_ang / GRID_RESOLUTION;
	double small_circle_y_4 = point.y + ABS_COLLISION_CIRCLE_SMALL_L_TO_CAR_FRONT_AXLE_4 * sin_ang / GRID_RESOLUTION;
	if(abs_safe_map[(int)round(small_circle_x_4)][(int)round(small_circle_y_4)] <= expanded_r + ABS_COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION) return false;
	return true;
}

bool DecisionFSM::completTask(){
	for(auto task_point: task_points)
		if(fabs(task_point.x - CAR_CEN_ROW) < 3 && fabs(task_point.y - CAR_CEN_COL) < 2 && fabs(task_point.angle.getDegree()) < 15)
		{
			Config::getInstance()->tasks.erase(Config::getInstance()->tasks.begin());
			return true;
		}
	return false;
}

//TiEV end
}

