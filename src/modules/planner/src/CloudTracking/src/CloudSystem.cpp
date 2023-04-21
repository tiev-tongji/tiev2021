
#include "CloudSystem.h"

#include "../proto/TJ_v2x.pb.h"

namespace TiEV {
C2VinfoTran convertC2Vinfo(const C2Vinfo &info) {
  C2VinfoTran converted_info;
  converted_info.speed         = static_cast<int>(info.speed * 1e2);
  converted_info.heading_angle = static_cast<int>(info.heading_angle * 1e2);
  converted_info.utmX          = static_cast<int>(info.utmX * 1e2);
  converted_info.utmY          = static_cast<int>(info.utmY * 1e2);
  auto now                     = std::chrono::system_clock::now();
  auto duration                = now.time_since_epoch();
  converted_info.timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  return converted_info;
}
std::string convertToStr(const C2VinfoTran &converted_info) {
  cn::seisys::v2x::pb::VsmData vsmData;
  cn::seisys::v2x::pb::Position3D *self_pos =
      new cn::seisys::v2x::pb::Position3D;
  self_pos->set_lon(converted_info.utmX);
  self_pos->set_lat(converted_info.utmY);
  vsmData.set_obuid("1");
  vsmData.set_allocated_pos(self_pos);
  vsmData.set_speed(converted_info.speed);
  vsmData.set_heading(converted_info.heading_angle);
  vsmData.set_timestamp(converted_info.timestamp);
  std::string converted_str;
  vsmData.SerializeToString(&converted_str);
  return converted_str;
}

void publish(const std::string &msg,mqtt::async_client& client_) {
  auto pubmsg = mqtt::make_message(TOPIC_SEND, msg.c_str());
  pubmsg->set_qos(QOS);
  client_.publish(pubmsg);
}
void sendToCloud(const C2Vinfo &info,mqtt::async_client& client_) {
  auto        converted_info = convertC2Vinfo(info);
  std::string converStr      = convertToStr(converted_info);
  publish(converStr,client_);
}



} // namespace TiEV