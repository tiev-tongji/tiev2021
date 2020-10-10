#ifndef _CONTROLLER_TOOLS_H
#define _CONTROLLER_TOOLS_H
#include <vector>
#include "stanley_tracking.h"

double distance(ControlPose pose_a, ControlPose pose_b);

void getRefPose(const std::vector<ControlPose> control_path, const int direction, ControlPose current_pose, ControlPose &ref_pose);
void getRefPose2(const std::vector<ControlPose> control_path, const double current_velocity, const double aim_dis_k, const double aim_dis_base, ControlPose &ref_pose);
void getRefPose3(const std::vector<ControlPose> control_path, const int direction, ControlPose current_pose, ControlPose &ref_pose);

double filter(double value, double old_value, double alfa);

double getRefVelocity(const std::vector<ControlPose> control_path, const double current_velocity, const double aim_dis_k, const double aim_dis_base);
#endif
