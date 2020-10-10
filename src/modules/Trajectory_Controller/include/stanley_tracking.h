#ifndef _STANLEY_TRACKING_H
#define _STANLEY_TRACKING_H
#include <vector>
// the refPose and currentPose in the ISO coradinate
//direction 1 is forward, backward is -1
//current vel is m/s and has +-
const double PI = 3.1415926;
struct ControlPose{
	double x;
	double y;
	double angle; //rad
	double kappa;
	double v; //m/s
};

class StanleyTracking{
public:
	void setVehicleParameter(double wheel_base, double mass,double lf, double lr, double cornering_stiffness, double max_wheel_angle);
	void setControlParameter(double k_lateral_front, double k_lareral_rear, double k_heading_dev, double k_lateral_front_high, double k_heading_dev_high, bool flag_dynamic, double k_kappa_dev);
	void computeWheelAngle(const ControlPose ref_pose, const ControlPose current_pose, double current_vel, int direction, double current_yawrate, double &wheelangle_cmd);
	void computeWheelAngleHighSpeed(const ControlPose ref_pose, const ControlPose current_pose, double current_vel, int direction, double current_yawrate, double &wheelangle_cmd);
		
private:
	double computePositionError(ControlPose ref_pose, ControlPose current_pose, int direction);
	double computeHeadingError(ControlPose ref_pose, ControlPose current_pose);
	void rearPoseToFrontPose(ControlPose &pose);
	void frontPoseToRearPose(ControlPose &pose);
	double sign(double data);

	double _wheel_base;
	double _lf;
	double _lr;
	bool _flag_dynamic;
	double _max_wheel_angle;//rad
	double _cornering_stiffness;
	double _k_lateral_front;
	double _k_lateral_rear;
	double _k_heading_dev;
	double _k_lateral_front_high;
	double _k_heading_dev_high;
	double _k_kappa_dev;
	double _mass;//kg
};
#endif
