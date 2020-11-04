#include "decision_view.h"
#include <math.h>
#include "utils/utils.h"
#include "common/nature.h"

namespace TiEV{
using namespace std;

void DecisionView::setViewInfo(ViewInfo const &_view_info){
	unique_lock<mutex> lck(vi_mtx);
	view_info = _view_info;
}

void DecisionView::draw(cv::Mat &planner_left_map, cv::Mat &planner_right_map){
	unique_lock<mutex> lck(vi_mtx);
	left_map = cv::Mat(TiEV::GRID_ROW, TIEV::GRID_COL, CV_8UC3, TiEV_BLACK);
	right_map = cv::Mat(TiEV::GRID_ROW, TIEV::GRID_COL, CV_8UC3, TiEV_BLACK);

	//draw lidar_map
	if(view_info.lidar_map.detected){
		for(int r = 0; r < TiEV::GRID_ROW; r++){
			for(int c = 0; c < TiEV::GRID_COL; c++){
				if(view_info.lidar_map.map[r][c] != 0) *left_map.ptr<cv::Vec3b>(r,c) = VEC_OBS_COLOR;
			}
		}
	}

	//draw dynamic
	if(view_info.dynamic_objs.detected){
		for(int i = 0; i < view_info.dynamic_objs.dynamic_obj_list.size(); i++){
			DynamicObj obj = view_info.dynamic_objs.dynamic_obj_list[i]; vector<cv::Point> points;
			for(int j = 0; j < obj.corners.size(); j++){
				int x = obj.corners[j].y;
				int y = obj.corners[j].x;
				points.push_back(cv::Point(x,y));
			}
			vector<cv::Point> path;
			for(int j = 0; j < obj.path.size(); j++){
				int x = obj.path[j].y;
				int y = obj.path[j].x;
				path.push_back(cv::Point(x,y));
			}
			if(obj.type == 1) dynamic_obj_color = CAR_COLOR;
			else if(obj.type == 2) dynamic_obj_color = PEDE_COLOR;
			else dynamic_obj_color = UOBJ_COLOR;
			cv::polylines(left_map, points, true, dynamic_obj_color);
			cv::polylines(left_map, path, false, dynamic_obj_color);
		}
	}

	//draw warning obj TODO

	//draw parking lots
	if(view_info.parking_lots.detected){
		for(int i = 0; i < view_info.parking_lots.parking_lot_list.size(); i++){
			ParkingLot lot = view_info.parking_lots.parking_lot_list[i];
			vector<cv::Point> points;
			points.push_back(cv::Point(lot.left_front.y,lot.left_front.x));
			points.push_back(cv::Point(lot.left_back.y,lot.left_back.x));
			points.push_back(cv::Point(lot.right_back.y,lot.right_back.x));
			points.push_back(cv::Point(lot.right_front.y,lot.right_front.x));
			points.push_back(cv::Point(lot.left_front.y,lot.left_front.x));
			points.push_back(cv::Point(lot.right_back.y,lot.right_back.x));
			points.push_back(cv::Point(lot.right_front.y,lot.right_front.x));
			points.push_back(cv::Point(lot.left_back.y,lot.left_back.x));
			cv::polylines(left_map, points, false, PARKING_LOT_COLOR);
		}
	}

	//draw lane lines
	if(view_info.lanes.detected){
		for(int i = 0; i < view_info.lanes.lane_list.size(); i++){
			Lane lane = view_info.lanes.lane_list[i];
			if(lane.stop_point.x > 0){
				cv::line(left_map, cv::Point(lane.left_line.points.back().y, lane.left_line.points.back().x), \
				cv::Point(lane.right_line.points.back().y, lane.right_line.points.back().x), LANE_LINE_COLOR, 2);
			}
			if(lane.left_line.type == LineType::SOLID){
				vector<cv::Point> line;
				for(int j = 0; j < lane.left_line.points.size(); j++){
					int x = lane.left_line.points[j].y;
					int y = lane.left_line.points[j].x;
					line.push_back(cv::Point(x,y));
				}
				cv::polylines(left_map, line, false, LANE_LINE_COLOR);
			}else{
				vector<cv::Point> line;
				for(int j = 0; j < lane.left_line.points.size(); j++){
					int x = lane.left_line.points[j].y;
					int y = lane.left_line.points[j].x;
					line.push_back(cv::Point(x,y));
				}
				for(int t = 0; t < line.size(); t += 20){
					vector<cv::Point> line_pice;
					for(int k = t; k < t + 10 && k < line.size(); k++){
						line_pice.push_back(line[k]);
					}
					cv::polylines(left_map, line_pice, false, LANE_LINE_COLOR);
				}
			}
			if(lane.right_line.type == LineType::SOLID){
				vector<cv::Point> line;
				for(int j = 0; j < lane.right_line.points.size(); j++){
					int x = lane.right_line.points[j].y;
					int y = lane.right_line.points[j].x;
					line.push_back(cv::Point(x,y));
				}
				cv::polylines(left_map, line, false, LANE_LINE_COLOR);
			}else{
				vector<cv::Point> line;
				for(int j = 0; j < lane.right_line.points.size(); j++){
					int x = lane.right_line.points[j].y;
					int y = lane.right_line.points[j].x;
					line.push_back(cv::Point(x,y));
				}
				for(int t = 0; t < line.size(); t += 20){
					vector<cv::Point> line_pice;
					for(int k = t; k < t + 10 && k < line.size(); k++){
						line_pice.push_back(line[k]);
					}
					cv::polylines(left_map, line_pice, false, LANE_LINE_COLOR);
				}
			}
		}
	}

	//draw reference path
	left_map.copyTo(right_map);

	for(int i = 0; i < view_info.reference_path.size(); i++){
		int x = view_info.reference_path[i].x;
		int y = view_info.reference_path[i].y;
		if(!utils::isInLocalMap(view_info.reference_path[i])) continue;
		*right_map.ptr<cv::Vec3b>(x,y) = VEC_REFER_PATH_COLOR;
	}

	//draw reference lane line
	for(int i = 0; i < view_info.all_road_infoes.size(); i++){
		double lane_width = view_info.all_road_infoes[i].lane_width;
		int lane_id = view_info.all_road_infoes[i].lane_seq;
		int lane_num = view_info.all_road_infoes[i].lane_num;
		if(0 == lane_num) continue;
		for(int j = 0; j < lane_num + 1; j++){
			Point lane_line_point = utils::getLateralPoint(view_info.all_reference_path[i], lane_width * (j - lane_id) + lane_width/2);
			int lx = lane_line_point.x;
			int ly = lane_line_point.y;
			if(lx < 0 || ly < 0 || lx >= GRID_ROW || ly >= GRID_COL) continue;
			*right_map.ptr<cv::Vec3b>(lx,ly) = VEC_HDMAP_LINE_COLOR;
		}
	}

	//draw best path
	for(int i = 0; i < view_info.best_path.size(); i++){
		int x = view_info.best_path[i].x;
		int y = view_info.best_path[i].y;
		if(!utils::isInLocalMap(view_info.best_path[i])) continue;
		*right_map.ptr<cv::Vec3b>(x,y) = VEC_BEST_PATH_COLOR;
	}

	//draw targets
	for(int i = 0; i < view_info.targets.size(); i++){
		int x = view_info.targets[i].y;
		int y = view_info.targets[i].x;
		cv::circle(right_map, cv::Point(x,y), 2, TARGET_COLOR, -1);
	}

	//draw start point
	cv::circle(right_map, cv::Point(view_info.start_point.y,view_info.start_point.x), 2, START_POINT_COLOR, -1);

	left_map.copyTo(planner_left_map);
	right_map.copyTo(planner_right_map);
}

void ViewInfo::clear(){
	road_infoes.clear();
	reference_path.clear();
	all_road_infoes.clear();
	all_reference_path.clear();
	best_path.clear();
	targets.clear();
	start_point.x = CAR_CEN_ROW;
	start_point.y = CAR_CEN_COL;

	nav_info.detected = false;
	lidar_map.detected = false;
	dynamic_objs.detected = false;
	warning_objs.detected = false;
	traffic_light.detected = false;
	parking_lots.detected = false;
	lanes.detected = false;
}

}//TiEV end
