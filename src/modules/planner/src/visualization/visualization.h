#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_
#include <map>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <string>
#include <vector>
#include <zcm/zcm-cpp.hpp>

#include "TiEV_colors.h"
#include "const.h"
#include "message_manager.h"
#include "pose.h"
#include "speed_optimizer.h"
#include "splines/Splines.h"
#include "tiev_utils.h"
#include "tievmsg.h"
#include "visualization_msg.h"

namespace TiEV {
using namespace std;
using namespace SplineLib;

static std::mutex vs_mtx;
static std::mutex text_mtx;
static std::mutex speed_mtx;

static std::mutex decision_mtx;
static std::mutex speed_planner_mtx;
static std::mutex path_planner_mtx;

struct Coefficient {
  vector<double> params;
  Coefficient() {}

  Coefficient(vector<double> coes) { params = coes; }

  double get_value(double x) {
    double result = 0.0;
    for (auto rit = params.rbegin(); rit != params.rend(); ++rit) {
      result *= x;
      result += (*rit);
    }
    return result;
  }
};

class Visualization {
 public:
  const double alpha     = 0.5;
  const double belta     = 1 - alpha;
  bool         TiEV_Stop = false;

 public:
  static Visualization* getInstance() {
    vs_mtx.lock();
    static Visualization instance;
    vs_mtx.unlock();
    return &instance;
  }

  void visualize();
  template <typename T>
  void print_text(const char* name, T const& value, int text_position = 0,
                  bool cut = true) {
    string text = to_string(value);
    if (cut) {
      size_t pos = text.rfind(".");
      if (pos != string::npos && text.length() - pos > 3) {
        text.erase(pos + 3, text.length());
      }
    }
    text_mtx.lock();
    switch (text_position) {
      case 1: {
        if (nav_info.find(name) != nav_info.end())
          nav_info[name] = text;
        else
          nav_info.insert(pair<string, string>(name, text));
        break;
      }
      case 2: {
        if (perception_info.find(name) != perception_info.end())
          perception_info[name] = text;
        else
          perception_info.insert(pair<string, string>(name, text));
        break;
      }
      default: {
        if (planner_info.find(name) != planner_info.end())
          planner_info[name] = text;
        else
          planner_info.insert(pair<string, string>(name, text));
      }
    }
    text_mtx.unlock();
  }

  void print_text(const char* name, const char* text, int text_position = 0);
  void set_speed_view(cv::Mat& speed_mat);
  void msgReceiveUdp();
  void msgReceiveIpc();
  void publishRemoteControl(const structREMOTECONTROL& remote_control);

 private:
  Visualization() { init(); };
  ~Visualization(){};

  void init();
  void clear();
  void draw_speed_window();
  void draw_text_window();
  void draw_planner_window();

 private:
  cv::Mat main_window;
  cv::Mat text_main_window;
  cv::Mat speed_view_main_window;
  cv::Mat planner_main_window;

  cv::Mat TiEV_car;
  cv::Mat traffic_light_rect;
  cv::Mat traffic_light_gray_straight;
  cv::Mat traffic_light_gray_left;
  cv::Mat traffic_light_gray_right;
  cv::Mat traffic_light_green_straight;
  cv::Mat traffic_light_green_left;
  cv::Mat traffic_light_green_right;
  cv::Mat traffic_light_red_straight;
  cv::Mat traffic_light_red_left;
  cv::Mat traffic_light_red_right;
  cv::Mat traffic_light_yellow_straight;
  cv::Mat traffic_light_yellow_left;
  cv::Mat traffic_light_yellow_right;
  cv::Mat auto_rect;
  cv::Mat auto_window;
  cv::Mat auto_start;
  cv::Mat auto_end;

  cv::Mat text_window;
  cv::Mat speed_view_window;
  cv::Mat planner_window;

  cv::Mat init_text_window;
  cv::Mat init_speed_view_window;
  cv::Mat init_planner_window;

  cv::Mat planner_map_left;
  cv::Mat planner_map_right;
  cv::Mat traffic_light_window;
  cv::Mat left_traffic_light_window;
  cv::Mat straight_traffic_light_window;
  cv::Mat right_traffic_light_window;

  map<string, string> nav_info;
  map<string, string> perception_info;
  map<string, string> planner_info;

  /*绘制PathPlanner相关信息*/
  void drawPathPlanner();
  /*绘制SpeedPlanner相关信息*/
  void drawSpeedPlanner(cv::Mat& speed_view_mat);
  /*获取可视化文字信息*/
  void getTextInfo();

  /*绘制可视化信息的函数*/
  bool drawTrafficLight();
  bool drawLidarMap(cv::Mat& left_map, cv::Mat& right_map,
                    int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawSafeMap(cv::Mat& left_map, cv::Mat& right_map,
                   int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawUsedMap(cv::Mat& left_map, cv::Mat& right_map,
                   int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawDynamicObjs(cv::Mat& left_map, cv::Mat& right_map,
                       int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawParkingLots(cv::Mat& left_map, cv::Mat& right_map,
                       int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawLanes(cv::Mat& left_map, cv::Mat& right_map,
                 int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawReferencePath(cv::Mat& left_map, cv::Mat& right_map,
                         int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawMaintainedPath(cv::Mat& left_map, cv::Mat& right_map, int opt);
  bool drawBestPath(cv::Mat& left_map, cv::Mat& right_map,
                    int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawTargets(cv::Mat& left_map, cv::Mat& right_map,
                   int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawStartPoint(cv::Mat& left_map, cv::Mat& right_map,
                      int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawPaths(cv::Mat& left_map, cv::Mat& right_map,
                 int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawReferenceLanes(cv::Mat& left_map, cv::Mat& right_map,
                          int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawPriorityLane(cv::Mat& left_map, cv::Mat& right_map,
                        int opt);  // opt: 0 for left, 1 for right, 2 for both
  bool drawSTBoundaries(cv::Mat& speed_view_mat);
  bool drawDPReferenceCurve(cv::Mat& speed_view_mat);
  bool drawQPSpeedCurve(cv::Mat& speed_view_mat);
  bool drawSplinesSpeedCurve(cv::Mat& speed_view_mat);

  const static time_t NAV_INFO_TIMEOUT_US      = 1e6;
  const static time_t SLAM_LOC_TIMEOUT_US      = 1e6;
  const static time_t LIDAR_MAP_TIMEOUT_US     = 1e6;
  const static time_t LANE_TIMEOUT_US          = 2e5;
  const static time_t TRAFFIC_LIGHT_TIMEOUT_US = 2e6;
  const static time_t PARKING_SLOT_TIMEOUT_US  = 1e6;
  const static time_t VISUALIZATION_TIMEOUT_US = 1e6;
  const static time_t OBJECT_LIST_TIMEOUT_US   = 1e6;
  const static int    OBJECTS_SOURCE_NUM       = 3;

  // zcm可视化消息获取
  class Handler {
   public:
    void handleNAVINFO(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const structNAVINFO* msg);
    void handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf,
                         const std::string& chan, const structFUSIONMAP* msg);
    void handleTRAFFICLIGHT(const zcm::ReceiveBuffer*    rbuf,
                            const std::string&           chan,
                            const MsgTrafficLightSignal* msg);
    void handleLANES(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const structRoadMarkingList* msg);
    void handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf,
                            const std::string&        chan,
                            const structPARKINGSLOTS* msg);
    void handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf,
                          const std::string& chan, const structOBJECTLIST* msg);
    void handleSLAMLOC(const zcm::ReceiveBuffer* rbuf, const std::string& chan,
                       const structSLAMLOC* msg);
    void handleVISUALIZATION(const zcm::ReceiveBuffer* rbuf,
                             const std::string&        chan,
                             const visVISUALIZATION*   msg);

    time_t update_time_nav_info;
    time_t update_time_slam_loc;
    time_t update_time_lidar;
    time_t update_time_objects[OBJECTS_SOURCE_NUM];
    time_t update_time_traffic_light;
    time_t update_time_lanes;
    time_t update_time_parking_slots;
    time_t update_time_visualization;

    std::shared_mutex nav_mtx, slam_loc_mtx, lidar_mtx, objects_mtx,
        traffic_mtx, lane_mtx, parking_lots_mtx, visualization_mtx;
    structNAVINFO         tmp_nav;
    structSLAMLOC         tmp_slam_loc;
    structFUSIONMAP       tmp_lidar_map;
    MsgTrafficLightSignal tmp_traffic;
    structOBJECTLIST      tmp_objects[OBJECTS_SOURCE_NUM];
    structRoadMarkingList tmp_lanes;
    structPARKINGSLOTS    tmp_slot;
    visVISUALIZATION      tmp_visualization;
  };

  Handler  inner_handler;
  zcm::ZCM zcm_ipc{"ipc"};
  zcm::ZCM zcm_udp{""};
};

}  // namespace TiEV
#endif
