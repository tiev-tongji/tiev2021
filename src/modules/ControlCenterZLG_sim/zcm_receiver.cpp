#include "zcm_receiver.h"
#include <iostream>

void Handler::handleCANCONTROLZLG(const zcm::ReceiveBuffer *rbuf, const std::string &channel,
                     const structCANCONTROLZLG *msg);
  controlinfo_mutex.lock();
  update_time_controlinfo = getTimeStamp();
  tmp_controlinfo = *msg;
  //	update_time_path = TiEV::getTimeStamp();
  controlinfo_mutex.unlock();
}



void ZcmReceiver::zcmMsgReceive() {
  zcm::ZCM zcm;
  if (!zcm.good()) {
    std::cout << "zcm is not good! " << std::endl;
    return;
  }

  zcm.subscribe("CANCONTROLZLG", &Handler::handleCANCONTROLZLG, &inner_handler);

  while (0 == zcm.handle())
    ;
}

bool ZcmReceiver::getCANControlInfo(int64_t &current_timestamp, double &current_throttle, double &current_steer, double &current_brake,int8_t &current_reverse) {
  if (getTimeStamp() - inner_handler.update_time_controlinfo > CANControlInfo_TIME_OUT_US)
    return false;

  inner_handler.controlinfo_mutex.lock();
  current_timestamp = inner_handler.tmp_controlinfo.timestamp;
  current_throttle = inner_handler.tmp_controlinfo.throttle;
  current_steer = inner_handler.tmp_controlinfo.steer;
  current_brake = inner_handler.tmp_controlinfo.brake;
  current_reverse = inner_handler.tmp_controlinfo.reverse;
  
  
    //		std::cout << "curvature[" << i << "]------------------------->"
    //<<
    // buffer_point.kappa << std::endl;
  
  inner_handler.controlinfo_mutex.unlock();

  return true;
}


