#include <cmath>
#include <iostream>
#include "controller_tools.h"

double distance(ControlPose pose_a, ControlPose pose_b){
	double result;
	result = sqrt((pose_a.x - pose_b.x)*(pose_a.x - pose_b.x) + (pose_a.y - pose_b.y)*(pose_a.y - pose_b.y));
	return result;
}

void getRefPose(const std::vector<ControlPose> control_path, const int direction, ControlPose current_pose, ControlPose &ref_pose){
	if(control_path.size() < 1) return;

	if(direction == -1){
		double wheel_base = 2.305;
		current_pose.x = current_pose.x - wheel_base * std::cos(current_pose.angle);
		current_pose.y = current_pose.y - wheel_base * std::sin(current_pose.angle);
	}

	int n = control_path.size();
	ref_pose = control_path[0];
	std::cout << "i = 0" << " x:" << ref_pose.x << " y:" << ref_pose.y << " angle:" << ref_pose.angle * 180 / 3.1415926536<< std::endl;
	for(int i = 1; i < n; i++){
		if(distance(current_pose, control_path[i]) < distance(current_pose, ref_pose)){
			ref_pose = control_path[i];
			std::cout << "i = " << i << " x:" << ref_pose.x << " y:" << ref_pose.y << " angle:" << ref_pose.angle * 180 / 3.1415926536<< std::endl;
		}
	}

}

void getRefPose2(const std::vector<ControlPose> control_path, const double current_velocity, const double aim_dis_k, const double aim_dis_base, ControlPose &ref_pose){
	if(control_path.size() < 1) return;

	int n = control_path.size();
	ref_pose = control_path[0];

	double aim_dis = aim_dis_k * current_velocity + aim_dis_base;
        std::cout << "aim_dis = " << aim_dis << std::endl;

	for(int i = 0; i < n; i++){
		if(control_path[i].x < aim_dis) continue;
        ref_pose = control_path[i];
        std::cout << "i = " << i << std::endl;
		break;
    }
}

void getRefPose3(const std::vector<ControlPose> control_path, const int direction, ControlPose current_pose, ControlPose &ref_pose){
	if(control_path.size() < 1) return;

	int n = control_path.size();
	if(direction == -1){
		double wheel_base = 2.305;
		current_pose.x = current_pose.x - wheel_base * std::cos(current_pose.angle);
		current_pose.y = current_pose.y - wheel_base * std::sin(current_pose.angle);

		std::vector<ControlPose> control_path_rear;
		for(int i = 0; i < n; i++){
			ControlPose tmp;
			tmp.x = control_path[i].x - wheel_base;
			tmp.y = control_path[i].y;
			tmp.angle = control_path[i].angle;
			tmp.v = control_path[i].v;
			tmp.kappa = control_path[i].kappa;
			control_path_rear.push_back(tmp);

//			std::cout << "i = " << i << " x:" << tmp.x << " y:" << tmp.y << " angle:" << tmp.angle * 180 / 3.1415926536<< std::endl;
		}
		ref_pose = control_path_rear[0];
		for(int i = 1; i < n; i++){
			if(distance(current_pose, control_path_rear[i]) < distance(current_pose, ref_pose)){
				ref_pose = control_path_rear[i];
				std::cout << "i = " << i << " x:" << ref_pose.x << " y:" << ref_pose.y << " angle:" << ref_pose.angle * 180 / 3.1415926536<< std::endl;
			}
		}
	}

	else{
		ref_pose = control_path[0];
		std::cout << "i = 0" << " x:" << ref_pose.x << " y:" << ref_pose.y << " angle:" << ref_pose.angle * 180 / 3.1415926536 << std::endl;
		for(int i = 1; i < n; i++){
			if(distance(current_pose, control_path[i]) < distance(current_pose, ref_pose)){
				ref_pose = control_path[i];
				std::cout << "i = " << i << " x:" << ref_pose.x << " y:" << ref_pose.y << " angle:" << ref_pose.angle * 180 / 3.1415926536<< std::endl;
			}
		}
	}
}

double filter(double value, double old_value, double alfa){
	double filter_value;
	filter_value = (1 - alfa) * old_value + alfa * value;
	return filter_value;
}

double getRefVelocity(const std::vector<ControlPose> control_path, const double current_velocity, const double aim_dis_k, const double aim_dis_base){
	double result;
	double aim_dis = aim_dis_k * current_velocity + aim_dis_base;
	int n = control_path.size();

	result = control_path[n-1].v;
	for(int i = 0; i < n; i++){
		if(fabs(control_path[i].x) < fabs(aim_dis)) continue;
		result = control_path[i].v;
		break;
	}
	return result;
}
