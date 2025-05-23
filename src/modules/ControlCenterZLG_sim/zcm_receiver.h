#ifndef _ZCM_RECEIVER_H_
#define _ZCM_RECEIVER_H_
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "structCANCONTROLZLG.hpp"
#include "timestamp.h"
#include <zcm/zcm-cpp.hpp>

class Handler {
public:
  Handler(){};
  ~Handler(){};

  void handleCANCONTROLZLG(const zcm::ReceiveBuffer *rbuf, const std::string &channel,
                     const structCANCONTROLZLG *msg);

  std::mutex controlinfo_mutex;

  time_t update_time_controlinfo;

  structCANCONTROLZLG tmp_controlinfo;
};

class ZcmReceiver {
public:
  ZcmReceiver(){};
  ~ZcmReceiver(){};

  bool getCANControlInfo(int64_t &current_timestamp, double &current_throttle, double &current_steer, double &current_brake,int8_t &current_reverse);
  void zcmMsgReceive();

private:
  time_t CANControlInfo_TIME_OUT_US = 1e6;
  Handler inner_handler;
};
#endif
