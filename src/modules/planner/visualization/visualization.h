#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <mutex>
#include <string>
#include <map>

#include "decision_view.h"
#include "path_planner_view.h"
#include "speed_view/speed_view.h"
#include "TiEV_colors.h"

namespace TiEV{


using namespace std;

static std::mutex vs_mtx;
static std::mutex text_mtx;
static std::mutex speed_mtx;

class Visualization{
	public:
		const double alpha = 0.5; const double belta = 1 - alpha;
		bool TiEV_Stop = false;
	public:
		static Visualization* getInstance(){
			vs_mtx.lock();
			static Visualization instance;
			vs_mtx.unlock();
			return &instance;
		}

		void visualize();
		template <typename T> void print_text(const char* name, T const& value, int text_position=0, bool cut = true){
			string text = to_string(value);
			if(cut){
				size_t pos = text.rfind(".");
				if(pos != string::npos && text.length() - pos > 3){
					text.erase(pos+3, text.length());
				}
			}
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

		void print_text(const char* name, const char* text, int text_position = 0);
		void set_speed_view(cv::Mat &speed_mat);

	private:
		Visualization(){
			init();
		};
		~Visualization(){};

		void init();
		void clear();
		void draw_speed_window();
		void draw_text_window();
		void draw_planner_window();
	
	private:
		
		cv::Mat main_window;
		cv::Mat text_main_window;
		cv::Mat speed_view_main_window;
		cv::Mat planner_main_window;

		cv::Mat TiEV_car;

		cv::Mat text_window;
		cv::Mat speed_view_window;
		cv::Mat planner_window;

		cv::Mat init_text_window;
		cv::Mat init_speed_view_window;
		cv::Mat init_planner_window;

		cv::Mat planner_map_left;
		cv::Mat planner_map_mid;
		cv::Mat planner_map_right;

		map<string, string> nav_info;
		map<string, string> perception_info;
		map<string, string> planner_info;

		DecisionView *dv;
		SpeedView *sv;
		PathPlannerView *ppv;
};

}//end TiEV
#endif
