#include "speed_view.h"
using namespace SplineLib;
namespace TiEV{

void SpeedView::init(){
	speed_mat = cv::Mat(500, 500, CV_8UC3, TiEV_BLACK);
}

void SpeedView::draw(cv::Mat &speed_view_map){
	speed_mat = cv::Mat(500, 500, CV_8UC3, TiEV_BLACK);

    unique_lock<std::mutex> lck(sp_mtx);
	//draw obstacles
	for(int i = 0; i < stbs.size(); i++){
		STBoundary stb = stbs[i];
		vector<cv::Point> points;
		points.push_back(cv::Point(stb.upper_left_point().t()/T_RESOLUTION, MAXS-stb.upper_left_point().s()/S_RESOLUTION));
		points.push_back(cv::Point(stb.upper_right_point().t()/T_RESOLUTION, MAXS-stb.upper_right_point().s()/S_RESOLUTION));
		points.push_back(cv::Point(stb.bottom_right_point().t()/T_RESOLUTION, MAXS-stb.bottom_right_point().s()/S_RESOLUTION));
		points.push_back(cv::Point(stb.bottom_left_point().t()/T_RESOLUTION, MAXS-stb.bottom_left_point().s()/S_RESOLUTION));
		cv::polylines(speed_mat, points, true, TiEV_RED);
	}

	/////
	for (const auto& speed_point : dp_speed_reference) {
        cv::circle(speed_mat, cv::Point(speed_point.t() / T_RESOLUTION, MAXS - speed_point.s() / S_RESOLUTION)
                , 3, cv::Scalar(0,0,255), -1);
	}

	//draw speed line from quntic curve
	for(int i = 0; i < coes.size(); i++){
		int area_size = (int)(500 / coes.size());
		co = coes[i];
		for(int t = 0; t < area_size; ++t){
		    double dt = t * T_RESOLUTION;
			double s = co.get_value(dt);
			int r = MAXS - s / S_RESOLUTION;
			int c = t + i * area_size;
			if(r < 0 || r >= MAXS || c < 0 || c >= MAXT) continue;
			*speed_mat.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLUE;
		}
	}

	//draw speed line from splines
	for(int i = 0; i < splines_num; i++){
		cSpline2 spline = splines[i];
		for(int t = 0; t < 50; t++){
			double dt = t * T_RESOLUTION * 2;
			Vec2f position = Position(spline, dt);
			int r = MAXS - position.y / S_RESOLUTION;
			int c = t + i * 50;
			if(r < 0 || r >= MAXS || c < 0 || c >= MAXT) continue;
			*speed_mat.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLUE;
		}
	}

	//add to Visualization
	speed_mat.copyTo(speed_view_map);
}

void SpeedView::setSpeedPath(const SpeedPath &speed_path){
    unique_lock<std::mutex> lck(sp_mtx);
	coes.clear();
	stbs.clear();
    dp_speed_reference = speed_path.dp_speed_data;
    splines_num = 0;
    if(!speed_path.success || speed_path.path.empty()) return;
	stbs = speed_path.st_boundaries;
	if(speed_path.qp_success){
		for(auto spline: speed_path.splines.splines()){
			coes.push_back(spline.spline_func().params());
		}
	}else{
		splines_num = speed_path.cubic_splines.size();
		for(int i = 0; i < splines_num; i++){
			splines[i] = speed_path.cubic_splines[i];
		}
	}
}

}//TiEV end
