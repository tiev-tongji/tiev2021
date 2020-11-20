#ifndef _ZCM_RECEIVER_H_
#define _ZCM_RECEIVER_H_
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "stanley_tracking.h"
#include "structAIMPATH.hpp"
#include "structCANINFO.hpp"
#include "structNAVINFO.hpp"
#include "timestamp.h"
#include <zcm/zcm-cpp.hpp>

class Handler {
public:
  Handler(){};
  ~Handler(){};

  void handleAIMPATH(const zcm::ReceiveBuffer *rbuf, const std::string &channel,
                     const structAIMPATH *msg);
  void handleCANINFO(const zcm::ReceiveBuffer *rbuf, const std::string &channel,
                     const structCANINFO *msg);
  void handleNAVINFO(const zcm::ReceiveBuffer *rbuf, const std::string &channel,
                     const structNAVINFO *msg);

  std::mutex path_mutex;
  std::mutex caninfo_mutex;
  std::mutex navinfo_mutex;

  time_t update_time_path;
  time_t update_time_caninfo;
  time_t update_time_navinfo;

  structAIMPATH tmp_path;
  structCANINFO tmp_caninfo;
  structNAVINFO tmp_navinfo;
};

class ZcmReceiver {
public:
  ZcmReceiver(){};
  ~ZcmReceiver(){};

  bool getControlPath(std::vector<ControlPose> &control_path);
  bool getCarStatus(double &current_velocity, double &current_steering_angle,
                    double &current_yawrate);
  void zcmMsgReceive();

private:
  time_t AIMPATH_TIME_OUT_US = 1e6;
  Handler inner_handler;
};
#endif
