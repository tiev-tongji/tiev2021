#ifndef _DECISION_VIEW_H
#define _DECISION_VIEW_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "message_manager.h"
#include "road_manager.h"
#include "TiEV_colors.h"

using namespace std;
namespace TiEV{

static std::mutex vi_mtx;
static std::mutex dv_mtx;

class ViewInfo{
	public:
		NavInfo nav_info;
		LidarMap lidar_map;
		DynamicObjList dynamic_objs;
		WarningObjList warning_objs;
		TrafficLight traffic_light;
		ParkingLotList parking_lots;
		LaneList lanes;

		vector<RoadInfo> road_infoes;
		vector<Point> reference_path;
		vector<RoadInfo> all_road_infoes;
		vector<Point> all_reference_path;
		vector<Point> best_path;
		vector<Point> targets;
		Point start_point;

	public:
		void clear();
};

class DecisionView{
	public:
		static DecisionView* getInstance(){
			dv_mtx.lock();
			static DecisionView instance;
			dv_mtx.unlock();
			return &instance;
		}

		void setViewInfo(ViewInfo const &_view_info);
		void draw(cv::Mat &left_map, cv::Mat &right_map);

	private:
		ViewInfo view_info;
		cv::Mat left_map;
		cv::Mat right_map;

		cv::Scalar dynamic_obj_color;
		cv::Scalar stop_line_color;

	private:
		DecisionView(){};
		~DecisionView(){};

};

}//TiEV end
#endif
