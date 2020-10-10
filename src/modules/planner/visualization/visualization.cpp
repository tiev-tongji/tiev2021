#include "visualization.h"
#include <iostream>
#include "config/config.h"

namespace TiEV{
using namespace std;

void Visualization::init(){
	string TiEV_logo_path = Config::getInstance()->TiEV_CONFIG_DIRECT + "pics/TiEV_logo.png";
	string TiEV_car_jpg_path = Config::getInstance()->TiEV_CONFIG_DIRECT + "pics/TiEV_car.jpg";
	string TiEV_car_png_path = Config::getInstance()->TiEV_CONFIG_DIRECT + "pics/TiEV_car.png";
	dv = DecisionView::getInstance();
	sv = SpeedView::getInstance();
	ppv = PathPlannerView::getInstance();

	main_window = cv::Mat(1000, 1200, CV_8UC3, TiEV_BLACK);

	text_main_window = main_window(cv::Rect(0,0,680,900));
	planner_main_window = main_window(cv::Rect(680,0,520,480));
	speed_view_main_window = main_window(cv::Rect(680,480,520,520));

	text_window = cv::Mat(900, 680, CV_8UC3, TiEV_BLACK);
	speed_view_window = cv::Mat(520, 520, CV_8UC3, TiEV_BLACK);
	planner_window = cv::Mat(480, 520, CV_8UC3, TiEV_BLACK);

	cv::putText(planner_window, "MAP1", cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, TiEV_BLUE, 2);
	cv::putText(planner_window, "MAP2", cv::Point(221, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, TiEV_BLUE, 2);
	cv::putText(planner_window, "MAP3", cv::Point(392, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, TiEV_BLUE, 2);

	init_text_window = cv::Mat(900,680,CV_8UC3,TiEV_BLACK);
	init_speed_view_window = cv::Mat(520,520,CV_8UC3,TiEV_BLACK);
	init_planner_window = cv::Mat(401,151,CV_8UC3,TiEV_BLUE);
	
	int font_style = cv::FONT_HERSHEY_SIMPLEX;
	double font_size = 1;
	cv::putText(init_planner_window, "No Data!", cv::Point(5,250), font_style, font_size, TiEV_BLACK, 2);
	cv::putText(init_speed_view_window, "No Data!", cv::Point(200, 350), font_style, font_size, TiEV_BLUE, 2);

	planner_map_left = planner_window(cv::Rect(13,60,151,401));
	planner_map_mid = planner_window(cv::Rect(184,60,151,401));
	planner_map_right = planner_window(cv::Rect(355,60,151,401));
	
	TiEV_car = cv::Mat(20, 9, CV_8UC3, TiEV_BLACK);
	
	cv::Mat car_img = cv::imread(TiEV_car_jpg_path);
	if(!car_img.data){
		cv::Mat car_img = cv::imread(TiEV_car_png_path);
		if(!car_img.data){
			cout << "no TiEV_car.jpg or TiEV_car.png in pics folder!!!" << endl;
		}else cv::resize(car_img, TiEV_car, TiEV_car.size(), 0, 0, INTER_LANCZOS4);
	}else cv::resize(car_img, TiEV_car, TiEV_car.size(), 0, 0, INTER_LANCZOS4);
	cv::Mat TiEV_logo = cv::imread(TiEV_logo_path);
	if(!TiEV_logo.data) cout << "no TiEV_logo.png in pics folder!!!" << endl;
	else{
		cv::Mat TiEV_logo_main_window = main_window(cv::Rect(0,1000-TiEV_logo.rows,TiEV_logo.cols,TiEV_logo.rows));	
		cv::Mat mask = cv::imread(TiEV_logo_path, 0);
		cv::bitwise_not(mask, mask);
		cv::threshold(mask, mask, 100, 255, cv::THRESH_BINARY);
		TiEV_logo.copyTo(TiEV_logo_main_window, mask);
	}
}

void Visualization::visualize(){
	cv::namedWindow("TiEV", cv::WINDOW_KEEPRATIO);
	//cv::namedWindow("TiEV", cv::WINDOW_AUTOSIZE);
	int key = -1;
	structREMOTECONTROL remote_control;
	remote_control.enabled = 0x00;
	while(!TiEV_Stop){
		clear();
		if(0x00 == remote_control.enabled) print_text("!CarContolMode", "Manul");
		else print_text("!CarContolMode", "Auto");
		draw_text_window();
		draw_planner_window();
		draw_speed_window();
		cv::imshow("TiEV", main_window);
		MessageManager::getInstance()->publishRemoteControl(remote_control);
		key = cv::waitKey(50);
		if('q' == key) TiEV_Stop = true;
		else if(27 == key) remote_control.enabled = 0x00;
		else if(32 == key) remote_control.enabled = 0x01;
	}
	cout << "Exit TiEV Visualization..." << endl;
	cv::destroyWindow("TiEV");
}

void Visualization::print_text(const char* name, const char* text, int text_position){
	text_mtx.lock();
	switch(text_position){
		case 1: {
			if(nav_info.find(name) != nav_info.end()) nav_info[name] = text;
			else nav_info.insert(pair<string, string>(name, text));
			break;
		}
		case 2: {
			if(perception_info.find(name) != perception_info.end()) perception_info[name] = text;
			else perception_info.insert(pair<string, string>(name, text));
			break;
		}
		default: {
			if(planner_info.find(name) != planner_info.end()) planner_info[name] = text;
			else planner_info.insert(pair<string, string>(name, text));
		}
	}
	text_mtx.unlock();
}

void Visualization::clear(){
	init_planner_window.copyTo(planner_map_left);
	init_planner_window.copyTo(planner_map_mid);
	init_planner_window.copyTo(planner_map_right);
	init_text_window.copyTo(text_window);
	init_speed_view_window.copyTo(speed_view_window);
}

void Visualization::draw_speed_window(){
	speed_mtx.lock();
	cv::Mat speed_mat_window = speed_view_window(cv::Rect(20,0,500,500));
	sv->draw(speed_mat_window);
	cv::arrowedLine(speed_view_window, cv::Point(20,500), cv::Point(20,0), TiEV_BLUE, 1, 8, 0, 0.02);
	cv::arrowedLine(speed_view_window, cv::Point(20,500), cv::Point(520,500), TiEV_BLUE, 1, 8, 0, 0.02);
	for(int i = 1; i < 10; i++){
		double time = 0.5 * i;
		string time_str = to_string(time);
		size_t pos = time_str.find(".");
		if(pos != string::npos && time_str.length() - pos > 2){
			time_str.erase(pos+2, time_str.length());
		}
		int s = i*10;
		cv::Point time_ori;
		cv::Point s_ori;
		time_ori.x = i*50;
		time_ori.y = 515;
		s_ori.x = 0;
		s_ori.y = 500 - i*50;
		int font_style = cv::FONT_HERSHEY_SIMPLEX;
		double font_size = 0.4;
		cv::putText(speed_view_window, time_str, time_ori, font_style, font_size, TiEV_BLUE);
		cv::putText(speed_view_window, to_string(s), s_ori, font_style, font_size, TiEV_BLUE);
	}
	speed_view_window.copyTo(speed_view_main_window);
	speed_mtx.unlock();
}

void Visualization::draw_text_window(){
	int font_style = cv::FONT_HERSHEY_SIMPLEX;
	double font_size = 0.7;
	cv::putText(text_window, "---NAV INFO---", cv::Point(2, 20), font_style, 0.7, TiEV_WHITE, 2);
	cv::putText(text_window, "---PERCEPTION INFO---", cv::Point(2, 400), font_style, 0.7, TiEV_WHITE, 2);
	cv::putText(text_window, "---PLANNER INFO---", cv::Point(340, 20), font_style, 0.7, TiEV_WHITE, 2);
	text_mtx.lock();
	map<string, string>::iterator it;
	int i = 0;
	for(it = nav_info.begin(); it != nav_info.end(); ++it, i++){
		string dp;
		dp.append(it->first);
		dp.append(":");
		dp.append(it->second);
		cv::putText(text_window, dp, cv::Point(2, (i+1)*30+25), font_style, font_size, TiEV_BLUE, 1.5);
	}
	i = 0;
	for(it = perception_info.begin(); it != perception_info.end(); ++it, i++){
		string dp;
		dp.append(it->first);
		dp.append(":");
		dp.append(it->second);
		cv::putText(text_window, dp, cv::Point(2, (i+1)*30+405), font_style, font_size, TiEV_BLUE);
	}
	i = 0;
	for(it = planner_info.begin(); it != planner_info.end(); ++it, i++){
		string dp;
		dp.append(it->first);
		dp.append(":");
		dp.append(it->second);
		cv::putText(text_window, dp, cv::Point(340, (i+1)*30+25), font_style, font_size, TiEV_BLUE, 1.5);
	}
	text_mtx.unlock();
	text_window.copyTo(text_main_window);
}

void Visualization::draw_planner_window(){
	dv->draw(planner_map_left, planner_map_right);
	ppv->draw(planner_map_mid);
	cv::Rect car_rect = cv::Rect(71, 295, 9, 20);
	cv::addWeighted(TiEV_car, alpha, planner_map_mid(car_rect), belta, 0, planner_map_mid(car_rect));
	//cv::addWeighted(TiEV_car, alpha, planner_map_right(car_rect), belta, 0, planner_map_right(car_rect));
	TiEV_car.copyTo(planner_map_left(car_rect));
	/*
	TiEV_car.copyTo(planner_map_mid(car_rect));
	TiEV_car.copyTo(planner_map_right(car_rect));
	cv::rectangle(planner_map_left, car_rect, TiEV_ORANGE);
	cv::rectangle(planner_map_mid, car_rect, TiEV_ORANGE);
	*/
	cv::rectangle(planner_map_right, car_rect, TiEV_ORANGE);
	cv::Rect window_rect = cv::Rect(0, 0, 151, 401);
	cv::rectangle(planner_map_left, window_rect, TiEV_WHITE);
	cv::rectangle(planner_map_mid, window_rect, TiEV_WHITE);
	cv::rectangle(planner_map_right, window_rect, TiEV_WHITE);
	planner_window.copyTo(planner_main_window);
}

}//TiEV end
