#ifndef _SPEED_VIEW_H_
#define _SPEED_VIEW_H_
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <mutex>
#include <math.h>
#include "path_time_graph/st_boundary.h"
#include "splines/Splines.h"
#include <vector>
#include "TiEV_colors.h"
#include "speed_optimizer.h"

using namespace std;
using namespace SplineLib;
namespace TiEV{

static std::mutex vew_mtx;
static std::mutex sp_mtx;

struct Coefficient{
	vector<double> params;
	Coefficient(){}

	Coefficient(vector<double> coes)  {
	    params = coes;
	}

	double get_value(double x) {
        double result = 0.0;
        for (auto rit = params.rbegin(); rit != params.rend(); ++rit) {
            result *= x;
            result += (*rit);
        }
        return result;
	}
};

const int MAXT = 500;
const int MAXS = 500;
const double S_RESOLUTION = 0.2;//0.2m = 1px
const double T_RESOLUTION = 0.01;//0.02s = 1px

class SpeedView{
	public:
		static SpeedView* getInstance(){
			vew_mtx.lock();
			static SpeedView instance;
			vew_mtx.unlock();
			return &instance;
		}

		void setSpeedPath(const SpeedPath &speed_path);
		void draw(cv::Mat &speed_view_map);
		

	private:
		SpeedView(){
			init();
		};
		~SpeedView(){};
		void init();

	private:
		cv::Mat speed_mat;

		Coefficient co;
		cSpline2 splines[10];
		int splines_num = 0;
		vector<Coefficient> coes;
		vector<STBoundary> stbs;
		SpeedData dp_speed_reference;
};

}//TiEV end
#endif
