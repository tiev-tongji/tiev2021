#pragma once 

#include <iostream>
#include <fstream>
#include <vector>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "Eigen/Core"
using namespace Eigen;
#include "structLASERMAP.hpp"
#include "common/nature.h"
namespace  TiEV{
	struct ScanLine{
		int id;							      //��(0,0)��ʼ˳ʱ����
		int state;					          //obstacle��0�� or inifinity��1�� or initial state��-1��
		cv::Point endpoint_ori;		          //�յ㣨����ǳ���
		cv::Point endpoint_access;		      //�յ㣨����ǳ���
		std::vector<cv::Point> linepoints_ori;     //���ϵ����ص�
		std::vector<cv::Point> linepoints_access;  //���ϵ����ص�
	};

	void generate_virtualscans(structLASERMAP grid, std::vector<Eigen::Vector3f> &virtusalscans);
}


