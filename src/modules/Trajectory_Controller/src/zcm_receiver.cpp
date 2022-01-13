#include "zcm_receiver.h"
#include <iostream>

void Handler::handleAIMPATH(const zcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const structAIMPATH *msg)
{
  path_mutex.lock();
  update_time_path = getTimeStamp();
  tmp_path = *msg;
  //	update_time_path = TiEV::getTimeStamp();
  path_mutex.unlock();
}

void Handler::handleCANINFO(const zcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const structCANINFO *msg)
{
  caninfo_mutex.lock();
  tmp_caninfo = *msg;
  //	update_time_caninfo = TiEV::getTimeStamp();
  caninfo_mutex.unlock();
}

void Handler::handleNAVINFO(const zcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const structNAVINFO *msg)
{
  navinfo_mutex.lock();
  tmp_navinfo = *msg;
  //	update_time_navinfo = TiEV::getTimeStamp();
  navinfo_mutex.unlock();
}

void ZcmReceiver::zcmMsgReceive()
{
  zcm::ZCM zcm;
  if (!zcm.good()) {
    std::cout << "zcm is not good! " << std::endl;
    return;
  }

  zcm.subscribe("AIMPATH", &Handler::handleAIMPATH, &inner_handler);
  zcm.subscribe("CANINFO", &Handler::handleCANINFO, &inner_handler);
  zcm.subscribe("NAVINFO", &Handler::handleNAVINFO, &inner_handler);

  while (0 == zcm.handle())
    ;
}

bool ZcmReceiver::getControlPath(std::vector<ControlPose> &control_path)
{
  if (inner_handler.tmp_path.num_points < 1 ||
      getTimeStamp() - inner_handler.update_time_path > AIMPATH_TIME_OUT_US)
    return false;

  inner_handler.path_mutex.lock();
  int num = inner_handler.tmp_path.num_points;
  control_path.clear();
  for (int i = 0; i < num; i++)
  {
    ControlPose buffer_point;
    buffer_point.x = inner_handler.tmp_path.points[i].x;
    buffer_point.y = inner_handler.tmp_path.points[i].y;
    buffer_point.angle = inner_handler.tmp_path.points[i].theta;
    buffer_point.kappa = inner_handler.tmp_path.points[i].k;
    buffer_point.v = inner_handler.tmp_path.points[i].v;
    control_path.push_back(buffer_point);
    //		std::cout << "curvature[" << i << "]------------------------->"
    //<<
    // buffer_point.kappa << std::endl;
  }
  inner_handler.path_mutex.unlock();

  return true;
}

bool ZcmReceiver::getCarStatus(double &current_velocity,
                               double &current_steering_angle,
                               double &current_yawrate)
{
  inner_handler.caninfo_mutex.lock();
  current_velocity = inner_handler.tmp_caninfo.carspeed / 100.0;
  current_steering_angle = inner_handler.tmp_caninfo.carsteer;
  inner_handler.caninfo_mutex.unlock();

  inner_handler.navinfo_mutex.lock();
  current_yawrate = inner_handler.tmp_navinfo.mAngularRateZ;
  inner_handler.navinfo_mutex.unlock();

  return true;
}
