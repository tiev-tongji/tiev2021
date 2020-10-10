#ifndef TRAFFICDIR_DETECTION_HPP
#define TRAFFICDIR_DETECTION_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "opencv/cv.hpp"

using namespace cv;
using namespace std;

enum LIGHTTYPE{RIGHT = 0, FORWARD, LEFT, UNKNOWN};

class TraffirDir{
public:
	LIGHTTYPE JudgeLightType(Mat detected, const bool& isGreen);
	bool isCircle(Mat CutImg);
private:
	LIGHTTYPE findLight(Mat srcImg, Mat grayImg, int color);
	void splitImage(Mat img, Mat &imgRed, Mat &imgGreen);

public:
	
private:
	const int thresh_red_min = 255 * 0.7;
	const int thresh_red_max = 256;
	const int thresh_green_min = 0;
	const int thresh_green_max = 255 * 0.2;
	const float gain_green_a = 1.0;//0.25
	const float gain_red_a = 1.2;//0.45
	
	//indiscriminable
	const float thresh_circle_min = 0.70;
	const float thresh_circle_max = 0.95;
	const float thresh_arrow_max = 0.7;
	const float thresh_arrow_min = 0.2;
	const float thresh_center_hollow = 0.4;
	
	//const float thresh_up_down_ratio = 1.2;
	const float thresh_left_right_ratio = 1.21;
	
	const int min_size = 10;
	const float w_vs_h = 1.3;
};

#endif
