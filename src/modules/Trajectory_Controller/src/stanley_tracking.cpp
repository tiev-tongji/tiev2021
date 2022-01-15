#include <cmath>
#include <iostream>
#include "stanley_tracking.h"

void StanleyTracking::setVehicleParameter(double wheel_base, double mass, double lf, double lr, double cornering_stiffness, double max_wheel_angle){
	_wheel_base = wheel_base;
	_mass = mass;
	_lf = lf;
	_lr = lr;
	_cornering_stiffness = cornering_stiffness;
	_max_wheel_angle = max_wheel_angle;
}

void StanleyTracking::setControlParameter(double k_lateral_front, double k_lareral_rear, double k_heading_dev, double k_lateral_front_high, double k_heading_dev_high, bool flag_dynamic, double k_kappa_dev){
	_k_lateral_front = k_lateral_front;
	_k_lateral_rear = k_lareral_rear;
	_k_heading_dev = k_heading_dev;
	_k_lateral_front_high = k_lateral_front_high;
	_k_heading_dev_high = k_heading_dev_high;
	_flag_dynamic = flag_dynamic;
	_k_kappa_dev = k_kappa_dev;
};

void StanleyTracking::rearPoseToFrontPose(ControlPose &pose){
	pose.x = pose.x + _wheel_base * std::cos(pose.angle);
	pose.y = pose.y + _wheel_base * std::sin(pose.angle);
}

void StanleyTracking::frontPoseToRearPose(ControlPose &pose){
	pose.x = pose.x - _wheel_base * std::cos(pose.angle);
	pose.y = pose.y - _wheel_base * std::sin(pose.angle);
}

double StanleyTracking::computePositionError(ControlPose ref_pose, ControlPose current_pose, int direction){
	double pos_error,dis;
	double dx, dy;
	if (direction == 1){
		dx = current_pose.x - ref_pose.x;
		dy = current_pose.y - ref_pose.y;
	}
	else{
		frontPoseToRearPose(current_pose);
		std::cout << "current x: " << current_pose.x << " current y: " << current_pose.y << std::endl;
		std::cout << "refpose x: " << ref_pose.x << " refpose y: " << ref_pose.y << std::endl;
		dx = current_pose.x - ref_pose.x;//ref_pose has been transformed into front axle in front 
		dy = current_pose.y - ref_pose.y;
	} //current_pose in TiEV is on its front axle
	pos_error = -1 * (dx * std::sin(ref_pose.angle) - dy * std::cos(ref_pose.angle));
	return pos_error;
}

double StanleyTracking::computeHeadingError(ControlPose ref_pose, ControlPose current_pose){
	double heading_error;
	heading_error = current_pose.angle - ref_pose.angle;
	if (std::fabs(heading_error) >= PI){
		heading_error = atan2(std::sin(heading_error), std::cos(heading_error));//-pi to pi
	}
	return heading_error;
}

void StanleyTracking::computeWheelAngle(const ControlPose ref_pose, const ControlPose current_pose, double current_vel, int direction, double current_yawrate, double &wheelangle_cmd){
    if(fabs(current_vel) < 0.1) return;

	double pos_error, heading_error;
	double k_soft = 1;
	double kinematic_gain;
	double dynamic_gain;
	double sum_gain;

	pos_error = computePositionError(ref_pose, current_pose, direction);
	heading_error = computeHeadingError(ref_pose, current_pose);
	if (direction == 1){
		kinematic_gain = (_k_heading_dev * heading_error + atan(_k_lateral_front * pos_error / (k_soft + std::fabs(current_vel))));
//		kinematic_gain = atan(1);
	}
	else{
		// kinematic_gain = -1 *(_k_heading_dev * heading_error + atan(-1 * _k_lateral_rear * pos_error / (k_soft + std::fabs(current_vel))));
		kinematic_gain = -1.0 *(_k_heading_dev * heading_error + atan(-1.0 * 0.1 * pos_error));
	}
	dynamic_gain = _k_kappa_dev * (current_vel * ref_pose.kappa - current_yawrate) + current_vel * current_vel * ref_pose.kappa * _mass / (2 * _cornering_stiffness * (1 + _lf / _lr));
	if (_flag_dynamic){
		sum_gain = kinematic_gain + dynamic_gain;
	}else{
		sum_gain = kinematic_gain;//rad 
	}
	wheelangle_cmd = sign(sum_gain) * std::fmin(std::fabs(sum_gain), _max_wheel_angle);

	std::cout << "pos_error = " << pos_error << "  heading_error = " << heading_error * 180 / PI << std::endl; 
}

void StanleyTracking::computeWheelAngleHighSpeed(const ControlPose ref_pose, const ControlPose current_pose, double current_vel, int direction, double current_yawrate, double &wheelangle_cmd){
    if(fabs(current_vel) < 0.1) return;

	double pos_error, heading_error;
	double k_soft = 1;
	double kinematic_gain;
	double dynamic_gain;
	double sum_gain;

	pos_error = computePositionError(ref_pose, current_pose, direction);
	heading_error = computeHeadingError(ref_pose, current_pose);
	if (direction == 1){
		kinematic_gain = (_k_heading_dev_high * heading_error + atan(_k_lateral_front_high * pos_error / (k_soft + std::fabs(current_vel))));
//		kinematic_gain = atan(1);
	}
	else{
		kinematic_gain = -1 *(_k_heading_dev * heading_error + atan(-1 * _k_lateral_rear * pos_error / (k_soft + std::fabs(current_vel))));
	}
	dynamic_gain = _k_kappa_dev * (current_vel * ref_pose.kappa - current_yawrate) + current_vel * current_vel * ref_pose.kappa * _mass / (2 * _cornering_stiffness * (1 + _lf / _lr));
	if (_flag_dynamic){
		sum_gain = kinematic_gain + dynamic_gain;
	}else{
		sum_gain = kinematic_gain;//rad 
	}
	wheelangle_cmd = sign(sum_gain) * std::fmin(std::fabs(sum_gain), _max_wheel_angle);

	std::cout << "pos_error = " << pos_error << "  heading_error = " << heading_error * 180 / PI << std::endl; 
}

double StanleyTracking::sign(double data){
	if (data > 0){
		return 1;
	}else if (data < 0){
		return -1;
	}else
	{
		return 0;
	}
}

