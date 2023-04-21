#include "visualization.h"

#include <iostream>

#include "config.h"
#include "const.h"
#include "tievlog.h"

namespace TiEV {
using namespace std;

void Visualization::init() {
  cout << "initialize visualization" << endl;
  string TiEV_cfg_pics_path =
      Config::getInstance().TiEV_CONFIG_DIRECT + "pics/";
  string TiEV_logo_path =
      Config::getInstance().TiEV_CONFIG_DIRECT + "pics/TiEV_logo.png";
  string TiEV_car_jpg_path =
      Config::getInstance().TiEV_CONFIG_DIRECT + "pics/TiEV_car.jpg";
  string TiEV_car_png_path =
      Config::getInstance().TiEV_CONFIG_DIRECT + "pics/TiEV_car.png";
  string TiEV_traffic_light_path =
      Config::getInstance().TiEV_CONFIG_DIRECT + "pics/traffic_light/";

  main_window = cv::Mat(900, 1100, CV_8UC3, TiEV_BLACK);

  speed_view_main_window = main_window(cv::Rect(0, 100, 520, 520));
  text_main_window       = main_window(cv::Rect(0, 620, 1100, 280));
  planner_main_window    = main_window(cv::Rect(520, 0, 580, 620));

  text_window       = cv::Mat(380, 1100, CV_8UC3, TiEV_BLACK);
  speed_view_window = cv::Mat(520, 520, CV_8UC3, TiEV_BLACK);
  planner_window    = cv::Mat(620, 580, CV_8UC3, TiEV_BLACK);

  // cv::putText(planner_window, "Perception", cv::Point(85, 80),
  // cv::FONT_HERSHEY_SIMPLEX, 0.7, TiEV_BLUE, 2); cv::putText(planner_window,
  // "Planning", cv::Point(390, 80), cv::FONT_HERSHEY_SIMPLEX, 0.7, TiEV_BLUE,
  // 2);

  init_text_window       = cv::Mat(280, 1100, CV_8UC3, TiEV_BLACK);
  init_speed_view_window = cv::Mat(520, 520, CV_8UC3, TiEV_BLACK);
  init_planner_window    = cv::Mat(MAX_ROW, MAX_COL, CV_8UC3, TiEV_BLUE);

  int    font_style = cv::FONT_HERSHEY_SIMPLEX;
  double font_size  = 1;
  cv::putText(init_planner_window, "No Data!", cv::Point(55, 250), font_style,
              font_size, TiEV_BLACK, 2);
  cv::putText(init_speed_view_window, "No Data!", cv::Point(200, 350),
              font_style, font_size, TiEV_BLUE, 2);

  planner_map_left  = planner_window(cv::Rect(20, 100, MAX_COL, MAX_ROW));
  planner_map_right = planner_window(cv::Rect(311, 100, MAX_COL, MAX_ROW));
  auto_rect         = cv::Mat(80, 80, CV_8UC3, TiEV_BLACK);
  auto_window =
      planner_window(cv::Rect(10, 10, auto_rect.cols, auto_rect.rows));
  traffic_light_rect        = cv::Mat(60, 60, CV_8UC3, TiEV_BLACK);
  traffic_light_window      = planner_window(cv::Rect(
      190, 20, 3 * traffic_light_rect.cols + 20, traffic_light_rect.rows));
  left_traffic_light_window = traffic_light_window(
      cv::Rect(0, 0, traffic_light_rect.cols, traffic_light_rect.rows));
  straight_traffic_light_window = traffic_light_window(
      cv::Rect(traffic_light_rect.cols + 10, 0, traffic_light_rect.cols,
               traffic_light_rect.rows));
  right_traffic_light_window = traffic_light_window(
      cv::Rect(traffic_light_rect.cols * 2 + 20, 0, traffic_light_rect.cols,
               traffic_light_rect.rows));

  TiEV_car = cv::Mat(20, 9, CV_8UC3, TiEV_BLACK);

  cv::Mat car_img = cv::imread(TiEV_car_jpg_path);
  auto_start      = cv::imread(TiEV_cfg_pics_path + "auto_start.jpg");
  auto_end        = cv::imread(TiEV_cfg_pics_path + "auto_end.jpg");
  traffic_light_gray_straight =
      cv::imread(TiEV_traffic_light_path + "gray_straight.png");
  traffic_light_gray_left =
      cv::imread(TiEV_traffic_light_path + "gray_left.png");
  traffic_light_gray_right =
      cv::imread(TiEV_traffic_light_path + "gray_right.png");
  traffic_light_green_straight =
      cv::imread(TiEV_traffic_light_path + "green_straight.png");
  traffic_light_green_left =
      cv::imread(TiEV_traffic_light_path + "green_left.png");
  traffic_light_green_right =
      cv::imread(TiEV_traffic_light_path + "green_right.png");
  traffic_light_red_straight =
      cv::imread(TiEV_traffic_light_path + "red_straight.png");
  traffic_light_red_left = cv::imread(TiEV_traffic_light_path + "red_left.png");
  traffic_light_red_right =
      cv::imread(TiEV_traffic_light_path + "red_right.png");
  cv::resize(auto_start, auto_start, auto_rect.size(), 0, 0,
             cv::INTER_LANCZOS4);
  cv::resize(auto_end, auto_end, auto_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  for (int r = 0; r < auto_rect.rows; ++r) {
    for (int c = 0; c < auto_rect.cols; ++c) {
      if ((*auto_start.ptr<cv::Vec3b>(r, c))[0] > 200)
        *auto_start.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if ((*auto_end.ptr<cv::Vec3b>(r, c))[0] > 200)
        *auto_end.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
    }
  }
  cv::resize(traffic_light_gray_left, traffic_light_gray_left,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_gray_straight, traffic_light_gray_straight,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_gray_right, traffic_light_gray_right,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_green_left, traffic_light_green_left,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_green_straight, traffic_light_green_straight,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_green_right, traffic_light_green_right,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_red_left, traffic_light_red_left,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_red_straight, traffic_light_red_straight,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::resize(traffic_light_red_right, traffic_light_red_right,
             traffic_light_rect.size(), 0, 0, cv::INTER_LANCZOS4);
  for (int r = 0; r < traffic_light_rect.rows; ++r) {
    for (int c = 0; c < traffic_light_rect.cols; ++c) {
      if (*traffic_light_gray_left.ptr<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
        *traffic_light_gray_left.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_gray_straight.ptr<cv::Vec3b>(r, c) ==
          cv::Vec3b(0, 0, 0))
        *traffic_light_gray_straight.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_gray_right.ptr<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
        *traffic_light_gray_right.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_green_left.ptr<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
        *traffic_light_green_left.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_green_straight.ptr<cv::Vec3b>(r, c) ==
          cv::Vec3b(0, 0, 0))
        *traffic_light_green_straight.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_green_right.ptr<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
        *traffic_light_green_right.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_red_left.ptr<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
        *traffic_light_red_left.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_red_straight.ptr<cv::Vec3b>(r, c) ==
          cv::Vec3b(0, 0, 0))
        *traffic_light_red_straight.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
      if (*traffic_light_red_right.ptr<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
        *traffic_light_red_right.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLACK;
    }
  }
  // if (!car_img.data) {
  //   cv::Mat car_img = cv::imread(TiEV_car_png_path);
  //   if (!car_img.data) {
  //     cout << "no TiEV_car.jpg or TiEV_car.png in pics folder!!!" << endl;
  //   } else
  //     cv::resize(car_img, TiEV_car, TiEV_car.size(), 0, 0, cv::INTER_LANCZOS4);
  // } else
    // cv::resize(car_img, TiEV_car, TiEV_car.size(), 0, 0, cv::INTER_LANCZOS4);
  cv::Mat TiEV_logo = cv::imread(TiEV_logo_path);
  if (!TiEV_logo.data)
    cout << "no TiEV_logo.png in pics folder!!!" << endl;
  else {
    cv::Mat TiEV_logo_main_window =
        main_window(cv::Rect(0, 0, TiEV_logo.cols, TiEV_logo.rows));
    cv::Mat mask = cv::imread(TiEV_logo_path, 0);
    cv::bitwise_not(mask, mask);
    cv::threshold(mask, mask, 100, 255, cv::THRESH_BINARY);
    TiEV_logo.copyTo(TiEV_logo_main_window, mask);
  }
  cout << "finish initialize visualization" << endl;
}

void Visualization::visualize() {
  cv::namedWindow("TiEV", cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
  cv::resizeWindow("TiEV", 1100, 900);
  int                 key = -1;
  structREMOTECONTROL remote_control;
  remote_control.enabled = 0x00;
  while (!TiEV_Stop) {
    clear();
    getTextInfo();
    draw_text_window();
    draw_planner_window();
    draw_speed_window();
    cv::imshow("TiEV", main_window);
    key = cv::waitKey(50);
    if ('q' == key) {
      TiEV_Stop              = true;
      remote_control.enabled = 0x00;
    } else if (27 == key)
      remote_control.enabled = 0x00;
    else if (32 == key) {
      remote_control.enabled = 0x01;
    }
    else if('b' == key){
      remote_control.enabled = 0x11;
      LOG(INFO) <<" 'b' to stop the car immly ";
    }
    if (remote_control.enabled)
      auto_start.copyTo(auto_window);
    else
      auto_end.copyTo(auto_window);


    publishRemoteControl(remote_control);
  }
  cout << "Exit TiEV Visualization..." << endl;
  cv::destroyWindow("TiEV");
}

void Visualization::print_text(const char* name, const char* text,
                               int text_position) {
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

void Visualization::clear() {
  init_planner_window.copyTo(planner_map_left);
  init_planner_window.copyTo(planner_map_right);
  init_text_window.copyTo(text_window);
  init_speed_view_window.copyTo(speed_view_window);
}

void Visualization::draw_speed_window() {
  speed_mtx.lock();
  cv::Mat speed_mat_window = speed_view_window(cv::Rect(20, 0, 500, 500));
  drawSpeedPlanner(speed_mat_window);
  cv::arrowedLine(speed_view_window, cv::Point(20, 500), cv::Point(20, 0),
                  TiEV_BLUE, 1, 8, 0, 0.02);
  cv::arrowedLine(speed_view_window, cv::Point(20, 500), cv::Point(520, 500),
                  TiEV_BLUE, 1, 8, 0, 0.02);
  for (int i = 1; i < 10; i++) {
    double time     = 0.5 * i;
    string time_str = to_string(time);
    size_t pos      = time_str.find(".");
    if (pos != string::npos && time_str.length() - pos > 2) {
      time_str.erase(pos + 2, time_str.length());
    }
    int       s = i * 10;
    cv::Point time_ori;
    cv::Point s_ori;
    time_ori.x        = i * 50;
    time_ori.y        = 515;
    s_ori.x           = 0;
    s_ori.y           = 500 - i * 50;
    int    font_style = cv::FONT_HERSHEY_SIMPLEX;
    double font_size  = 0.4;
    cv::putText(speed_view_window, time_str, time_ori, font_style, font_size,
                TiEV_BLUE);
    cv::putText(speed_view_window, to_string(s), s_ori, font_style, font_size,
                TiEV_BLUE);
  }
  speed_view_window.copyTo(speed_view_main_window);
  speed_mtx.unlock();
}

void Visualization::draw_text_window() {
  int    font_style = cv::FONT_HERSHEY_SIMPLEX;
  double font_size  = 0.7;
  cv::putText(text_window, "---NAV INFO---", cv::Point(2, 20), font_style, 0.7,
              TiEV_WHITE, 2);
  cv::putText(text_window, "---PERCEPTION INFO---", cv::Point(400, 20),
              font_style, 0.7, TiEV_WHITE, 2);
  cv::putText(text_window, "---PLANNER INFO---", cv::Point(800, 20), font_style,
              0.7, TiEV_WHITE, 2);
  text_mtx.lock();
  map<string, string>::iterator it;
  int                           i = 0;
  for (it = nav_info.begin(); it != nav_info.end(); ++it, i++) {
    string dp;
    dp.append(it->first);
    dp.append(":");
    dp.append(it->second);
    cv::putText(text_window, dp, cv::Point(2, (i + 1) * 30 + 25), font_style,
                font_size, TiEV_BLUE, 1);
  }
  i = 0;
  for (it = perception_info.begin(); it != perception_info.end(); ++it, i++) {
    string dp;
    dp.append(it->first);
    dp.append(":");
    dp.append(it->second);
    cv::putText(text_window, dp, cv::Point(400, (i + 1) * 30 + 25), font_style,
                font_size, TiEV_BLUE);
  }
  i = 0;
  for (it = planner_info.begin(); it != planner_info.end(); ++it, i++) {
    string dp;
    dp.append(it->first);
    dp.append(":");
    dp.append(it->second);
    cv::putText(text_window, dp, cv::Point(800, (i + 1) * 30 + 25), font_style,
                font_size, TiEV_BLUE, 1.5);
  }
  text_mtx.unlock();
  text_window.copyTo(text_main_window);
}

void Visualization::draw_planner_window() {
  drawTrafficLight();
  drawPathPlanner();
  cv::Rect car_rect = cv::Rect(CAR_CEN_COL - 4, CAR_CEN_ROW - 5, 9, 20);
  // cv::addWeighted(TiEV_car, alpha, planner_map_right(car_rect), belta, 0,
  // planner_map_right(car_rect));
  TiEV_car.copyTo(planner_map_left(car_rect));
  /*
  TiEV_car.copyTo(planner_map_mid(car_rect));
  TiEV_car.copyTo(planner_map_right(car_rect));
  cv::rectangle(planner_map_left, car_rect, TiEV_ORANGE);
  cv::rectangle(planner_map_mid, car_rect, TiEV_ORANGE);
  */
  cv::rectangle(planner_map_right, car_rect, TiEV_ORANGE);
  cv::Rect window_rect = cv::Rect(0, 0, MAX_COL, MAX_ROW);
  cv::rectangle(planner_map_left, window_rect, TiEV_WHITE);
  cv::rectangle(planner_map_right, window_rect, TiEV_WHITE);
  planner_window.copyTo(planner_main_window);
}

void Visualization::getTextInfo() {
  time_t current_time = getTimeStamp();
  inner_handler.nav_mtx.lock_shared();
  if (current_time - inner_handler.update_time_nav_info < NAV_INFO_TIMEOUT_US) {
    print_text("utmX", inner_handler.tmp_nav.utmX, 1, false);
    print_text("utmY", inner_handler.tmp_nav.utmY, 1, false);
    print_text("heading", inner_handler.tmp_nav.mHeading, 1, false);
    print_text("speed(m/s)", inner_handler.tmp_nav.mSpeed3d, 1);
    print_text("speed(km/h)", inner_handler.tmp_nav.mSpeed3d * 3.6, 1);
    print_text("Location Signal", "RTK", 1);
  } else {
    print_text("utmX", "None", 1);
    print_text("utmY", "None", 1);
    print_text("heading", "None", 1);
    print_text("speed(m/s)", "None", 1);
    print_text("speed(km/h)", "None", 1);
    print_text("Location Signal", "None", 1);
  }
  inner_handler.nav_mtx.unlock_shared();

  inner_handler.slam_loc_mtx.lock_shared();
  if (current_time - inner_handler.update_time_slam_loc < SLAM_LOC_TIMEOUT_US) {
    print_text("utmX", inner_handler.tmp_slam_loc.x, 1, false);
    print_text("utmY", inner_handler.tmp_slam_loc.y, 1, false);
    print_text("heading", inner_handler.tmp_slam_loc.mHeading, 1, false);
    print_text("Location Signal", "SLAM", 1);
  }
  inner_handler.slam_loc_mtx.unlock_shared();

  inner_handler.lane_mtx.lock_shared();
  if (current_time - inner_handler.update_time_lanes < LANE_TIMEOUT_US) {
    print_text("lane line", "detected", 2);
  } else {
    print_text("lane line", "None", 2);
  }
  inner_handler.lane_mtx.unlock_shared();

  inner_handler.parking_lots_mtx.lock_shared();
  if (current_time - inner_handler.update_time_parking_slots <
      PARKING_SLOT_TIMEOUT_US) {
    print_text("parking spot", "detected", 2);
  } else {
    print_text("parking spot", "None", 2);
  }
  inner_handler.parking_lots_mtx.unlock_shared();

  inner_handler.visualization_mtx.lock_shared();
  if (current_time - inner_handler.update_time_visualization <
      VISUALIZATION_TIMEOUT_US) {
    planner_info.clear();
    for (const auto& text_info : inner_handler.tmp_visualization.text_info) {
      print_text(text_info.name.c_str(), text_info.value.c_str());
    }
  } else {
    planner_info.clear();
    print_text("planner info", "None");
  }
  inner_handler.visualization_mtx.unlock_shared();
}

bool Visualization::drawTrafficLight() {
  inner_handler.traffic_mtx.lock_shared();
  time_t current_time = getTimeStamp();
  if (current_time - inner_handler.update_time_traffic_light <
      TRAFFIC_LIGHT_TIMEOUT_US) {
    if (inner_handler.tmp_traffic.left)
      traffic_light_green_left.copyTo(left_traffic_light_window);
    else
      traffic_light_red_left.copyTo(left_traffic_light_window);
    if (inner_handler.tmp_traffic.forward)
      traffic_light_green_straight.copyTo(straight_traffic_light_window);
    else
      traffic_light_red_straight.copyTo(straight_traffic_light_window);
    if (inner_handler.tmp_traffic.right)
      traffic_light_green_right.copyTo(right_traffic_light_window);
    else
      traffic_light_red_right.copyTo(right_traffic_light_window);
  } else {
    traffic_light_gray_left.copyTo(left_traffic_light_window);
    traffic_light_gray_straight.copyTo(straight_traffic_light_window);
    traffic_light_gray_right.copyTo(right_traffic_light_window);
  }
  inner_handler.traffic_mtx.unlock_shared();
  return true;
}

// 绘制Decision相关信息
void Visualization::drawPathPlanner() {
  time_t  current_time = getTimeStamp();
  cv::Mat left_map(MAX_ROW, MAX_COL, CV_8UC3, TiEV_BLACK);
  cv::Mat right_map(MAX_ROW, MAX_COL, CV_8UC3, TiEV_BLACK);
  // 雷达地图
  bool lidar = drawLidarMap(left_map, right_map, 0);
  // 动态障碍物
  bool dynamic = drawDynamicObjs(left_map, right_map, 2);
  // 停车库位
  bool parking_spot = drawParkingLots(left_map, right_map, 0);
  // 视觉车道线
  bool lanes     = drawLanes(left_map, right_map, 0);
  bool visualize = false;
  if (current_time - inner_handler.update_time_visualization <
      VISUALIZATION_TIMEOUT_US) {
    inner_handler.visualization_mtx.lock_shared();
    drawSafeMap(left_map, right_map, 1);
    // 参考路
    drawReferencePath(left_map, right_map, 0);
    // 最优路径
    // drawBestPath(left_map, right_map, 0);
    // 目标点
    drawTargets(left_map, right_map, 1);
    // Maintained path
    drawMaintainedPath(left_map, right_map, 0);
    // 规划起始点
    drawStartPoint(left_map, right_map, 1);
    // used_map
    drawUsedMap(left_map, right_map, 1);
    // 规划路线
    drawPaths(left_map, right_map, 1);
    // 地图参考车道
    drawReferenceLanes(left_map, right_map, 0);
    drawPriorityLane(left_map, right_map, 1);
    inner_handler.visualization_mtx.unlock_shared();
    visualize = true;
  }
  if (lidar || dynamic || parking_spot || lanes || visualize) {
    left_map.copyTo(planner_map_left);
    right_map.copyTo(planner_map_right);
  }
}

void Visualization::drawSpeedPlanner(cv::Mat& speed_view_mat) {
  time_t current_time = getTimeStamp();
  if (current_time - inner_handler.update_time_visualization <
      VISUALIZATION_TIMEOUT_US) {
    inner_handler.visualization_mtx.lock_shared();
    cv::Mat speed_mat(500, 500, CV_8UC3, TiEV_BLACK);
    // st_boundaries
    drawSTBoundaries(speed_mat);
    // DP reference curve
    drawDPReferenceCurve(speed_mat);
    // speed curve
    if (inner_handler.tmp_visualization.qp_or_splines == 0)
      drawQPSpeedCurve(speed_mat);
    else
      drawSplinesSpeedCurve(speed_mat);

    speed_mat.copyTo(speed_view_mat);
    inner_handler.visualization_mtx.unlock_shared();
  }
}

// 绘制雷达地图
bool Visualization::drawLidarMap(cv::Mat& left_map, cv::Mat& right_map,
                                 int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  time_t current_time = getTimeStamp();
  if (current_time - inner_handler.update_time_lidar < LIDAR_MAP_TIMEOUT_US) {
    inner_handler.lidar_mtx.lock_shared();
    for (int r = 0; r < MAX_ROW; r++) {
      for (int c = 0; c < MAX_COL; c++) {
        if (inner_handler.tmp_lidar_map.cells[r][c] != 0) {
          if (opt == 0)
            *left_map.ptr<cv::Vec3b>(r, c) = VEC_OBS_COLOR;
          else if (opt == 1)
            *right_map.ptr<cv::Vec3b>(r, c) = VEC_OBS_COLOR;
          else {
            *left_map.ptr<cv::Vec3b>(r, c)  = VEC_OBS_COLOR;
            *right_map.ptr<cv::Vec3b>(r, c) = VEC_OBS_COLOR;
          }
        }
      }
    }
    inner_handler.lidar_mtx.unlock_shared();
    return true;
  }
  return false;
}

// 绘制规划地图
bool Visualization::drawSafeMap(cv::Mat& left_map, cv::Mat& right_map,
                                int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  // 绘制规划地图
  for (int r = 0; r < MAX_ROW; r++) {
    for (int c = 0; c < MAX_COL; c++) {
      if (inner_handler.tmp_visualization.safe_map[r][c] == 0) {
        if (opt == 0)
          *left_map.ptr<cv::Vec3b>(r, c) = VEC_OBS_COLOR;
        else if (opt == 1)
          *right_map.ptr<cv::Vec3b>(r, c) = VEC_OBS_COLOR;
        else {
          *left_map.ptr<cv::Vec3b>(r, c)  = VEC_OBS_COLOR;
          *right_map.ptr<cv::Vec3b>(r, c) = VEC_OBS_COLOR;
        }
      }
    }
  }
  return true;
}

// 绘制used_map
bool Visualization::drawUsedMap(cv::Mat& left_map, cv::Mat& right_map,
                                int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  // 绘制规划地图
  for (int r = 0; r < MAX_ROW; r++) {
    for (int c = 0; c < MAX_COL; c++) {
      if (inner_handler.tmp_visualization.used_map[r][c] != 0) {
        if (opt == 0)
          *left_map.ptr<cv::Vec3b>(r, c) = USED_COLOR;
        else if (opt == 1)
          *right_map.ptr<cv::Vec3b>(r, c) = USED_COLOR;
        else {
          *left_map.ptr<cv::Vec3b>(r, c)  = USED_COLOR;
          *right_map.ptr<cv::Vec3b>(r, c) = USED_COLOR;
        }
      }
    }
  }
  return true;
}

// 绘制动态障碍物
bool Visualization::drawDynamicObjs(cv::Mat& left_map, cv::Mat& right_map,
                                    int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  constexpr double y_offset     = 0;
  time_t           current_time = getTimeStamp();
  inner_handler.objects_mtx.lock_shared();
  for (int i = 0; i < OBJECTS_SOURCE_NUM; ++i) {
    // std::cout << "SizeOfDynamic: " << inner_handler.tmp_objects[i].obj.size()
    // << std::endl;
    if (current_time - inner_handler.update_time_objects[i] <
        OBJECT_LIST_TIMEOUT_US) {
      for (const auto& obj : inner_handler.tmp_objects[i].obj) {
        vector<cv::Point> points;
        int x1 = CAR_CEN_ROW - obj.corners.p1.x / GRID_RESOLUTION;
        int y1 = CAR_CEN_COL - obj.corners.p1.y / GRID_RESOLUTION;
        points.emplace_back(cv::Point(y1, x1));
        int x2 = CAR_CEN_ROW - obj.corners.p2.x / GRID_RESOLUTION;
        int y2 = CAR_CEN_COL - obj.corners.p2.y / GRID_RESOLUTION;
        points.emplace_back(cv::Point(y2, x2));
        int x3 = CAR_CEN_ROW - obj.corners.p3.x / GRID_RESOLUTION;
        int y3 = CAR_CEN_COL - obj.corners.p3.y / GRID_RESOLUTION;
        points.emplace_back(cv::Point(y3, x3));
        int x4 = CAR_CEN_ROW - obj.corners.p4.x / GRID_RESOLUTION;
        int y4 = CAR_CEN_COL - obj.corners.p4.y / GRID_RESOLUTION;
        points.emplace_back(cv::Point(y4, x4));

        vector<cv::Point> path;
        for (int j = 0; j < obj.path.size(); ++j) {
          int x = CAR_CEN_ROW - obj.path[j].x / GRID_RESOLUTION;
          int y = CAR_CEN_COL - obj.path[j].y / GRID_RESOLUTION;
          path.emplace_back(cv::Point(y, x));
        }
        cv::Scalar dynamic_obj_color;
        if (obj.obj_type == 1)
          dynamic_obj_color = CAR_COLOR;
        else if (obj.obj_type == 2)
          dynamic_obj_color = PEDE_COLOR;
        else
          dynamic_obj_color = UOBJ_COLOR;
        if (opt == 0) {
          cv::polylines(left_map, points, true, dynamic_obj_color);
          cv::polylines(left_map, path, false, dynamic_obj_color);
        } else if (opt == 1) {
          cv::polylines(right_map, points, true, dynamic_obj_color);
          cv::polylines(right_map, path, false, dynamic_obj_color);
        } else {
          cv::polylines(left_map, points, true, dynamic_obj_color);
          cv::polylines(left_map, path, false, dynamic_obj_color);
          cv::polylines(right_map, points, true, dynamic_obj_color);
          cv::polylines(right_map, path, false, dynamic_obj_color);
        }
      }
    }
  }
  inner_handler.objects_mtx.unlock_shared();
  return true;
}

// 绘制停车库位
bool Visualization::drawParkingLots(cv::Mat& left_map, cv::Mat& right_map,
                                    int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  time_t current_time = getTimeStamp();
  if (current_time - inner_handler.update_time_parking_slots <
      PARKING_SLOT_TIMEOUT_US) {
    inner_handler.parking_lots_mtx.lock_shared();
    for (const auto& parking_lot : inner_handler.tmp_slot.parking_slots) {
      int right_front_x =
          CAR_CEN_ROW - parking_lot.front_right.y / GRID_RESOLUTION;
      int right_front_y =
          CAR_CEN_COL + parking_lot.front_right.x / GRID_RESOLUTION;
      int left_front_x =
          CAR_CEN_ROW - parking_lot.front_left.y / GRID_RESOLUTION;
      int left_front_y =
          CAR_CEN_COL + parking_lot.front_left.x / GRID_RESOLUTION;
      int left_back_x = CAR_CEN_ROW - parking_lot.rear_left.y / GRID_RESOLUTION;
      int left_back_y = CAR_CEN_COL + parking_lot.rear_left.x / GRID_RESOLUTION;
      int right_back_x =
          CAR_CEN_ROW - parking_lot.rear_right.y / GRID_RESOLUTION;
      int right_back_y =
          CAR_CEN_COL + parking_lot.rear_right.x / GRID_RESOLUTION;
      vector<cv::Point> points;
      points.push_back(cv::Point(left_front_y, left_front_x));
      points.push_back(cv::Point(left_back_y, left_back_x));
      points.push_back(cv::Point(right_back_y, right_back_x));
      points.push_back(cv::Point(right_front_y, right_front_x));
      points.push_back(cv::Point(left_front_y, left_front_x));
      points.push_back(cv::Point(right_back_y, right_back_x));
      points.push_back(cv::Point(right_front_y, right_front_x));
      points.push_back(cv::Point(left_back_y, left_back_x));
      if (opt == 0)
        cv::polylines(left_map, points, false, PARKING_LOT_COLOR);
      else if (opt == 1)
        cv::polylines(right_map, points, false, PARKING_LOT_COLOR);
      else {
        cv::polylines(left_map, points, false, PARKING_LOT_COLOR);
        cv::polylines(right_map, points, false, PARKING_LOT_COLOR);
      }
    }
    inner_handler.parking_lots_mtx.unlock_shared();
    return true;
  }
  return false;
}

// 绘制视觉检测车道线
bool Visualization::drawLanes(cv::Mat& left_map, cv::Mat& right_map, int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  time_t current_time = getTimeStamp();
  if (current_time - inner_handler.update_time_lanes < LANE_TIMEOUT_US) {
    inner_handler.lane_mtx.lock_shared();
    // draw stop line
    if (inner_handler.tmp_lanes.stop_line.exist) {
      vector<cv::Point> vis_stop_line;
      for (const auto& stop_point :
           inner_handler.tmp_lanes.stop_line.stop_points) {
        int x = CAR_CEN_ROW - stop_point.x / GRID_RESOLUTION;
        int y = CAR_CEN_COL - stop_point.y / GRID_RESOLUTION;
        vis_stop_line.push_back(cv::Point(y, x));
      }
      if (opt == 0)
        cv::polylines(left_map, vis_stop_line, false, LANE_LINE_COLOR);
      else if (opt == 1)
        cv::polylines(right_map, vis_stop_line, false, LANE_LINE_COLOR);
      else {
        cv::polylines(left_map, vis_stop_line, false, LANE_LINE_COLOR);
        cv::polylines(right_map, vis_stop_line, false, LANE_LINE_COLOR);
      }
    }

    for (const auto& lane : inner_handler.tmp_lanes.lanes) {
      if (lane.left_line.line_type == 1 ||
          lane.left_line.line_type == 3) {  // 左车道线为虚线
        vector<cv::Point> line;
        for (const auto& point : lane.left_line.points) {
          int x = CAR_CEN_ROW - point.x / GRID_RESOLUTION;
          int y = CAR_CEN_COL - point.y / GRID_RESOLUTION;
          if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
          line.emplace_back(cv::Point(y, x));
        }
        for (int t = 0; t < line.size(); t += 20) {
          vector<cv::Point> line_pice;
          for (int k = t; k < t + 10 && k < line.size(); k++) {
            line_pice.push_back(line[k]);
          }
          if (opt == 0)
            cv::polylines(left_map, line_pice, false, LANE_LINE_COLOR);
          else if (opt == 1)
            cv::polylines(right_map, line_pice, false, LANE_LINE_COLOR);
          else {
            cv::polylines(left_map, line_pice, false, LANE_LINE_COLOR);
            cv::polylines(right_map, line_pice, false, LANE_LINE_COLOR);
          }
        }
      } else {  // 左车道线为实线
        vector<cv::Point> line;
        for (const auto& point : lane.left_line.points) {
          int x = CAR_CEN_ROW - point.x / GRID_RESOLUTION;
          int y = CAR_CEN_COL - point.y / GRID_RESOLUTION;
          if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
          line.emplace_back(cv::Point(y, x));
        }
        if (opt == 0)
          cv::polylines(left_map, line, false, LANE_LINE_COLOR);
        else if (opt == 1)
          cv::polylines(right_map, line, false, LANE_LINE_COLOR);
        else {
          cv::polylines(left_map, line, false, LANE_LINE_COLOR);
          cv::polylines(right_map, line, false, LANE_LINE_COLOR);
        }
      }
      if (lane.right_line.line_type == 1 ||
          lane.right_line.line_type == 3) {  // 右车道线为虚线
        vector<cv::Point> line;
        for (const auto& point : lane.right_line.points) {
          int x = CAR_CEN_ROW - point.x / GRID_RESOLUTION;
          int y = CAR_CEN_COL - point.y / GRID_RESOLUTION;
          if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
          line.emplace_back(cv::Point(y, x));
        }
        for (int t = 0; t < line.size(); t += 20) {
          vector<cv::Point> line_pice;
          for (int k = t; k < t + 10 && k < line.size(); k++) {
            line_pice.push_back(line[k]);
          }
          if (opt == 0)
            cv::polylines(left_map, line_pice, false, LANE_LINE_COLOR);
          else if (opt == 1)
            cv::polylines(right_map, line_pice, false, LANE_LINE_COLOR);
          else {
            cv::polylines(left_map, line_pice, false, LANE_LINE_COLOR);
            cv::polylines(right_map, line_pice, false, LANE_LINE_COLOR);
          }
        }
      } else {  // 右车道线为实线
        vector<cv::Point> line;
        for (const auto& point : lane.right_line.points) {
          int x = CAR_CEN_ROW - point.x / GRID_RESOLUTION;
          int y = CAR_CEN_COL - point.y / GRID_RESOLUTION;
          if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
          line.emplace_back(cv::Point(y, x));
        }
        if (opt == 0)
          cv::polylines(left_map, line, false, LANE_LINE_COLOR);
        else if (opt == 1)
          cv::polylines(right_map, line, false, LANE_LINE_COLOR);
        else {
          cv::polylines(left_map, line, false, LANE_LINE_COLOR);
          cv::polylines(right_map, line, false, LANE_LINE_COLOR);
        }
      }
    }
    inner_handler.lane_mtx.unlock_shared();
    return true;
  }
  return false;
}

// 绘制参考路
bool Visualization::drawReferencePath(cv::Mat& left_map, cv::Mat& right_map,
                                      int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  for (const auto& point : inner_handler.tmp_visualization.reference_path) {
    int x = point.x;
    int y = point.y;
    if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
    if (opt == 0)
      *left_map.ptr<cv::Vec3b>(x, y) = VEC_REFER_PATH_COLOR;
    else if (opt == 1)
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_REFER_PATH_COLOR;
    else {
      *left_map.ptr<cv::Vec3b>(x, y)  = VEC_REFER_PATH_COLOR;
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_REFER_PATH_COLOR;
    }
  }
  return true;
}

// 绘制最优路径
bool Visualization::drawBestPath(cv::Mat& left_map, cv::Mat& right_map,
                                 int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  for (const auto& point : inner_handler.tmp_visualization.best_path) {
    int x = point.x;
    int y = point.y;
    if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
    if (opt == 0)
      *left_map.ptr<cv::Vec3b>(x, y) = VEC_BEST_PATH_COLOR;
    else if (opt == 1)
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_BEST_PATH_COLOR;
    else {
      *left_map.ptr<cv::Vec3b>(x, y)  = VEC_BEST_PATH_COLOR;
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_BEST_PATH_COLOR;
    }
  }
  return true;
}

// 绘制目标点
bool Visualization::drawTargets(cv::Mat& left_map, cv::Mat& right_map,
                                int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  for (const auto& target : inner_handler.tmp_visualization.targets) {
    int x = target.y;
    int y = target.x;
    if (opt == 0)
      cv::circle(left_map, cv::Point(x, y), 2, TARGET_COLOR, -1);
    else if (opt == 1)
      cv::circle(right_map, cv::Point(x, y), 2, TARGET_COLOR, -1);
    else {
      cv::circle(left_map, cv::Point(x, y), 2, TARGET_COLOR, -1);
      cv::circle(right_map, cv::Point(x, y), 2, TARGET_COLOR, -1);
    }
  }
  return true;
}

bool Visualization::drawMaintainedPath(cv::Mat& left_map, cv::Mat& right_map,
                                       int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  for (const auto& point : inner_handler.tmp_visualization.maintained_path) {
    int x = point.x;
    int y = point.y;
    if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
    if (opt == 0)
      *left_map.ptr<cv::Vec3b>(x, y) = VEC_MAINTAINED_PATH_COLOR;
    else if (opt == 1)
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_MAINTAINED_PATH_COLOR;
    else {
      *left_map.ptr<cv::Vec3b>(x, y)  = VEC_MAINTAINED_PATH_COLOR;
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_MAINTAINED_PATH_COLOR;
    }
  }
  return true;
}

// 绘制规划起始点
bool Visualization::drawStartPoint(cv::Mat& left_map, cv::Mat& right_map,
                                   int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  int x = inner_handler.tmp_visualization.start_point.x;
  int y = inner_handler.tmp_visualization.start_point.y;
  if (opt == 0) cv::circle(left_map, cv::Point(y, x), 2, START_POINT_COLOR, -1);
  if (opt == 1)
    cv::circle(right_map, cv::Point(y, x), 2, START_POINT_COLOR, -1);
  else {
    cv::circle(left_map, cv::Point(y, x), 2, START_POINT_COLOR, -1);
    cv::circle(right_map, cv::Point(y, x), 2, START_POINT_COLOR, -1);
  }
  return true;
}

// 绘制所有规划路径
bool Visualization::drawPaths(cv::Mat& left_map, cv::Mat& right_map, int opt) {
  assert(opt >= 0);
  assert(opt <= 2);

  const cv::Vec3b PATH_COLORS[] = {
      cv::Vec3b(0xff, 0xaa, 0xaa), cv::Vec3b(0xaa, 0xff, 0xaa),
      cv::Vec3b(0xaa, 0xaa, 0xff), cv::Vec3b(0xff, 0xff, 0xaa),
      cv::Vec3b(0xaa, 0xff, 0xff)};
  for (const auto& point : inner_handler.tmp_visualization.planner_path.path) {
    int x = point.x;
    int y = point.y;
    if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
    if (opt == 0)
      *left_map.ptr<cv::Vec3b>(x, y) = PATH_COLORS[0];
    else if (opt == 1)
      *right_map.ptr<cv::Vec3b>(x, y) = PATH_COLORS[2];
    else {
      *left_map.ptr<cv::Vec3b>(x, y)  = PATH_COLORS[1];
      *right_map.ptr<cv::Vec3b>(x, y) = PATH_COLORS[3];
    }
  }
  return true;
}

// 绘制参考车道线
bool Visualization::drawReferenceLanes(cv::Mat& left_map, cv::Mat& right_map,
                                       int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  for (const auto& lane_line : inner_handler.tmp_visualization.lanes) {
    for (const auto& point : lane_line.lane_line_points) {
      int x = point.x;
      int y = point.y;
      if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
      if (opt == 0)
        *left_map.ptr<cv::Vec3b>(x, y) = VEC_HDMAP_LINE_COLOR;
      else if (opt == 1)
        *right_map.ptr<cv::Vec3b>(x, y) = VEC_HDMAP_LINE_COLOR;
      else {
        *left_map.ptr<cv::Vec3b>(x, y)  = VEC_HDMAP_LINE_COLOR;
        *right_map.ptr<cv::Vec3b>(x, y) = VEC_HDMAP_LINE_COLOR;
      }
    }
  }
  return true;
}

bool Visualization::drawPriorityLane(cv::Mat& left_map, cv::Mat& right_map,
                                     int opt) {
  assert(opt >= 0);
  assert(opt <= 2);
  const auto& priority_lane = inner_handler.tmp_visualization.priority_lane;
  for (const auto& p : priority_lane.origin_points) {
    int x = p.x;
    int y = p.y;
    if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
    if (opt == 0)
      *left_map.ptr<cv::Vec3b>(x, y) = VEC_ORIGIN_LANE_COLOR;
    else if (opt == 1)
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_ORIGIN_LANE_COLOR;
    else {
      *left_map.ptr<cv::Vec3b>(x, y)  = VEC_ORIGIN_LANE_COLOR;
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_ORIGIN_LANE_COLOR;
    }
  }
  for (const auto& p : priority_lane.priority_points) {
    int x = p.x;
    int y = p.y;
    if (x < 0 || x >= MAX_ROW || y < 0 || y >= MAX_COL) continue;
    if (opt == 0)
      *left_map.ptr<cv::Vec3b>(x, y) = VEC_PIORITY_LANE_COLOR;
    else if (opt == 1)
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_PIORITY_LANE_COLOR;
    else {
      *left_map.ptr<cv::Vec3b>(x, y)  = VEC_PIORITY_LANE_COLOR;
      *right_map.ptr<cv::Vec3b>(x, y) = VEC_PIORITY_LANE_COLOR;
    }
  }
  return true;
}

// 绘制速度规划st_boundaries
bool Visualization::drawSTBoundaries(cv::Mat& speed_view_mat) {
  for (const auto& stb : inner_handler.tmp_visualization.st_boundaries) {
    vector<cv::Point> points;
    points.push_back(cv::Point(stb.ulp.t / 0.01, 500 - stb.ulp.s / 0.2));
    points.push_back(cv::Point(stb.urp.t / 0.01, 500 - stb.urp.s / 0.2));
    points.push_back(cv::Point(stb.brp.t / 0.01, 500 - stb.brp.s / 0.2));
    points.push_back(cv::Point(stb.blp.t / 0.01, 500 - stb.blp.s / 0.2));
    cv::polylines(speed_view_mat, points, true, TiEV_RED);
  }
  return true;
}

// 绘制速度规划DP参考曲线
bool Visualization::drawDPReferenceCurve(cv::Mat& speed_view_mat) {
  for (const auto& speed_point : inner_handler.tmp_visualization.dp_speed) {
    cv::circle(speed_view_mat,
               cv::Point(speed_point.t / 0.01, 500 - speed_point.s / 0.2), 3,
               cv::Scalar(0, 0, 255), -1);
  }
  return true;
}

bool Visualization::drawQPSpeedCurve(cv::Mat& speed_view_mat) {
  for (int i = 0; i < inner_handler.tmp_visualization.qp_speed_curve.size();
       ++i) {
    int area_size =
        (int)(500 / inner_handler.tmp_visualization.qp_speed_curve.size());
    Coefficient co =
        Coefficient(inner_handler.tmp_visualization.qp_speed_curve[i].params);
    for (int t = 0; t < area_size; ++t) {
      double dt = t * 0.01;
      double s  = co.get_value(dt);
      int    r  = 500 - s / 0.2;
      int    c  = t + i * area_size;
      if (r < 0 || r >= 500 || c < 0 || c >= 500) continue;
      *speed_view_mat.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLUE;
    }
  }
  return true;
}

bool Visualization::drawSplinesSpeedCurve(cv::Mat& speed_view_mat) {
  for (int i = 0;
       i < inner_handler.tmp_visualization.splines_speed_curve.size(); ++i) {
    const auto& spline = inner_handler.tmp_visualization.splines_speed_curve[i];
    SplineLib::cSpline2 s;
    SplineLib::Vec4f    vec1 =
        SplineLib::Vec4f(spline.xb.x, spline.xb.y, spline.xb.z, spline.xb.w);
    SplineLib::Vec4f vec2 =
        SplineLib::Vec4f(spline.yb.x, spline.yb.y, spline.yb.z, spline.yb.w);
    s.xb = vec1;
    s.yb = vec2;
    for (int t = 0; t < 50; t++) {
      double dt       = t * 0.01 * 2;
      Vec2f  position = Position(s, dt);
      int    r        = 500 - position.y / 0.2;
      int    c        = t + i * 50;
      if (r < 0 || r >= 500 || c < 0 || c >= 500) continue;
      *speed_view_mat.ptr<cv::Vec3b>(r, c) = VEC_TiEV_BLUE;
    }
  }
  return true;
}

void Visualization::msgReceiveUdp() {
  if (!zcm_udp.good()) return;

  zcm_udp.subscribe("FUSIONMAP", &Handler::handleFUSIONMAP, &inner_handler);
  zcm_udp.subscribe("NAVINFO", &Handler::handleNAVINFO, &inner_handler);
  zcm_udp.subscribe("OBJECTLIST", &Handler::handleOBJECTLIST, &inner_handler);
  zcm_udp.subscribe("MsgTrafficLightSignal", &Handler::handleTRAFFICLIGHT,
                    &inner_handler);
  zcm_udp.subscribe("LANE_info", &Handler::handleLANES, &inner_handler);
  zcm_udp.subscribe("PARKINGSLOTS", &Handler::handlePARKINGSLOTS,
                    &inner_handler);
  zcm_udp.subscribe("SLAMLOC", &Handler::handleSLAMLOC, &inner_handler);
  zcm_ipc.subscribe("VISUALIZATION", &Handler::handleVISUALIZATION,
                    &inner_handler);

  zcm_udp.run();
}

void Visualization::msgReceiveIpc() {
  if (!zcm_ipc.good()) return;

  zcm_ipc.subscribe("VISUALIZATION", &Handler::handleVISUALIZATION,
                    &inner_handler);

  zcm_ipc.run();
}

void Visualization::publishRemoteControl(
    const structREMOTECONTROL& remote_control) {
  zcm_udp.publish("REMOTECONTROL", &remote_control);
}

void Visualization::Handler::handleFUSIONMAP(const zcm::ReceiveBuffer* rbuf,
                                             const std::string&        chan,
                                             const structFUSIONMAP*    msg) {
  lidar_mtx.lock();
  tmp_lidar_map     = *msg;
  update_time_lidar = getTimeStamp();
  lidar_mtx.unlock();
}

void Visualization::Handler::handleTRAFFICLIGHT(
    const zcm::ReceiveBuffer* rbuf, const std::string& chan,
    const MsgTrafficLightSignal* msg) {
  traffic_mtx.lock();
  tmp_traffic               = *msg;
  update_time_traffic_light = getTimeStamp();
  traffic_mtx.unlock();
}

void Visualization::Handler::handleLANES(const zcm::ReceiveBuffer*    rbuf,
                                         const std::string&           chan,
                                         const structRoadMarkingList* msg) {
  lane_mtx.lock();
  tmp_lanes         = *msg;
  update_time_lanes = getTimeStamp();
  lane_mtx.unlock();
}

void Visualization::Handler::handlePARKINGSLOTS(const zcm::ReceiveBuffer* rbuf,
                                                const std::string&        chan,
                                                const structPARKINGSLOTS* msg) {
  parking_lots_mtx.lock();
  tmp_slot                  = *msg;
  update_time_parking_slots = getTimeStamp();
  parking_lots_mtx.unlock();
}

void Visualization::Handler::handleOBJECTLIST(const zcm::ReceiveBuffer* rbuf,
                                              const std::string&        chan,
                                              const structOBJECTLIST*   msg) {
  objects_mtx.lock();
  tmp_objects[msg->data_source]         = *msg;
  update_time_objects[msg->data_source] = getTimeStamp();
  objects_mtx.unlock();
}

void Visualization::Handler::handleVISUALIZATION(const zcm::ReceiveBuffer* rbuf,
                                                 const std::string&        chan,
                                                 const visVISUALIZATION* msg) {
  visualization_mtx.lock();
  tmp_visualization         = *msg;
  update_time_visualization = getTimeStamp();
  visualization_mtx.unlock();
}

void Visualization::Handler::handleSLAMLOC(const zcm::ReceiveBuffer* rbuf,
                                           const std::string&        chan,
                                           const structSLAMLOC*      msg) {
  slam_loc_mtx.lock();
  tmp_slam_loc         = *msg;
  update_time_slam_loc = getTimeStamp();
  slam_loc_mtx.unlock();
}

void Visualization::Handler::handleNAVINFO(const zcm::ReceiveBuffer* rbuf,
                                           const std::string&        chan,
                                           const structNAVINFO*      msg) {
  nav_mtx.lock();
  tmp_nav              = *msg;
  update_time_nav_info = getTimeStamp();
  nav_mtx.unlock();
}
}  // namespace TiEV
