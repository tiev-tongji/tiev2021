#include "path_planner.h"

#include <termio.h>

#include <iostream>
#include <random>
#include <thread>

#include "collision_check.h"
#include "config.h"
#include "const.h"
#include "log.h"
#include "opencv2/opencv.hpp"
#include "pose.h"
#include "steering_functions/hc_cc_state_space/cc00_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc00_reeds_shepp_state_space.hpp"

using namespace std;
using namespace TiEV;
using namespace cv;

enum OperationStatus {
  WaitingForTargetPoint,
  WaitingForTargetAngle,
  Planning,
};

namespace TiEV {
#ifdef DEBUG_EXPANSION_CALLBACK
extern PathPlanner::node_expansion_callback node_expanded;
#endif
#ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
extern PathPlanner::analytic_expansion_callback analytic_expanded;
#endif
}  // namespace TiEV

OperationStatus operationStatus = WaitingForTargetPoint;
bool            target_preset   = false;
TiEV::Pose      targetPose;
char*           map_file_path    = NULL;
bool            backward_enabled = false;
double          current_speed    = 0;
cv::Mat         map_view, expansion_layer, analytic_expansion_layer;
#define mtov(a) ((a)*1.5)
#define vtom(a) ((a) / 1.5)

void usage() {
  cerr << "path_planner_test [-b] [-s SPEED] [-t TARGETX TARGETY TARGETA] "
          "MAP_FILE"
       << endl;
  cerr << "\t-b  --enbale-backward  Enable backwarding search." << endl;
  cerr << "\t-s  --speed            Set the speed of the vehicle in m/s."
       << endl;
  cerr << "\t-t  --target           Set the target pose of the vehicle" << endl;
  exit(0);
}

void read_args(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-b") == 0 ||
        strcmp(argv[i], "--enable-backward") == 0) {
      log_0("backward enabled");
      backward_enabled = true;
    } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--speed") == 0) {
      if (i + 1 < argc) {
        current_speed = atof(argv[++i]);
        log_0("current speed set: ", current_speed, " m/s");
      } else {
        cerr << "error using flag " << argv[i] << " , argument missing."
             << endl;
        usage();
      }
    } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--target") == 0) {
      if (i + 3 < argc) {
        targetPose.x   = atof(argv[++i]);
        targetPose.y   = atof(argv[++i]);
        targetPose.ang = atof(argv[++i]);
        target_preset  = true;
        log_0("target preset");
      } else {
        cerr << "error using flag " << argv[i] << " , argument missing."
             << endl;
        usage();
      }
    } else if (map_file_path == NULL) {
      map_file_path = argv[i];
      log_0("map file path: ", map_file_path);
    } else {
      cerr << "unrecognized input " << argv[i] << endl;
      usage();
    }
  }
}

void merge_layer(cv::Mat& dest, const cv::Mat& layer, double alpha) {
  if (layer.size != dest.size) return;
  for (int i = 0; i < layer.rows; ++i)
    for (int j = 0; j < layer.cols; ++j) {
      const auto& v = layer.at<Vec3b>(i, j);
      if (v[0] > 0 || v[1] > 0 || v[2] > 0) {
        dest.at<Vec3b>(i, j) *= (1 - alpha);
        dest.at<Vec3b>(i, j) += v * alpha;
      }
    }
}

void refresh_main_view() {
  cv::Mat main_view = map_view.clone();
  merge_layer(main_view, expansion_layer, 0.8);
  merge_layer(main_view, analytic_expansion_layer, 0.2);
  cv::imshow("PathPlanner Test", main_view);
  // while (cv::waitKey(0) != 32) ;
  cv::waitKey(1);
}

void on_mouse(int event, int x, int y, int flags, void* ustc) {
  switch (operationStatus) {
    case WaitingForTargetPoint: {
      if (event == CV_EVENT_LBUTTONDOWN) {
        targetPose.x    = vtom(y);
        targetPose.y    = vtom(x);
        operationStatus = WaitingForTargetAngle;
      }
      break;
    }
    case WaitingForTargetAngle: {
      if (event == CV_EVENT_LBUTTONUP) {
        targetPose.ang  = atan2(vtom(x) - targetPose.y, vtom(y) - targetPose.x);
        operationStatus = Planning;
      }
      break;
    }
  }
}

void on_node_expanded(double x, double y, double a, double k, bool is_backward,
                      double heuristic, double cost) {
  double     p = min(1.0, max(0.0, heuristic / 500.0));
  cv::Scalar colora(255, 0, 0), colorb(0, 0, 255);
  cv::Scalar color = colora + (colorb - colora) * p;
  cv::arrowedLine(expansion_layer, mtov(cv::Point(y, x)),
                  mtov(cv::Point(y + 8 * sin(a), x + 8 * cos(a))), color, 1, 8,
                  0, 0.2);
  return;
  // log_0("expansion: (x, y, a) = (", x, ", ", y, ", ", a, "), ",
  //     "curvature = ", k, ", ", (is_backward ? "backwarding" : "forwarding"),
  //     ", h = ", heuristic, ", g = ", cost);
  refresh_main_view();
}

void on_analytic_expanded(const vector<pair<double, double>>& xys) {
  return;
  analytic_expansion_layer = 0.0;
  for (const auto& xy_pair : xys)
    analytic_expansion_layer.at<cv::Vec3b>(
        mtov(lround(xy_pair.first)), mtov(lround(xy_pair.second))) = {200, 0,
                                                                      200};
  // log_0("analytic expansion failed, states = ", xys.size());
  refresh_main_view();
}

void generate_safe_map(double safe_map[MAX_ROW][MAX_COL]) {
  constexpr double max_obstacle_distance =
      COLLISION_CIRCLE_BIG_R / GRID_RESOLUTION + 5.0 / GRID_RESOLUTION;
#define vec(a, b) \
  { {a, b}, sqrt(a* a + b * b) }
  constexpr pair<pair<int, int>, double> deltas[] = {
      vec(-2, -1), vec(-2, 1), vec(-1, -2), vec(-1, -1), vec(-1, 0), vec(-1, 1),
      vec(-1, 2),  vec(0, -1), vec(0, 1),   vec(1, -2),  vec(1, -1), vec(1, 0),
      vec(1, 1),   vec(1, 2),  vec(2, -1),  vec(2, 1),
  };
  constexpr int deltas_length = sizeof(deltas) / sizeof(deltas[0]);
#undef vec
  time_t                time_0 = getTimeStamp();
  queue<pair<int, int>> que;
  bool                  in_queue[MAX_ROW][MAX_COL] = {0};
  for (int row = 0; row < MAX_ROW; ++row)
    for (int column = 0; column < MAX_COL; ++column) {
      safe_map[row][column] = min(safe_map[row][column], max_obstacle_distance);
      if (safe_map[row][column] == 0.0) {
        que.emplace(row, column);
        in_queue[row][column] = true;
      }
    }
  while (!que.empty()) {
    auto& front = que.front();
    int   x = front.first, y = front.second;
    que.pop();
    in_queue[x][y]  = false;
    double safe_x_y = safe_map[x][y];
    for (int i = 0; i < deltas_length; ++i) {
      int tx = x + deltas[i].first.first;
      int ty = y + deltas[i].first.second;
      if (tx >= 0 && tx < MAX_ROW && ty >= 0 && ty < MAX_COL) {
        double new_dis = safe_x_y + deltas[i].second;
        if (safe_map[tx][ty] > new_dis) {
          safe_map[tx][ty] = new_dis;
          if (safe_map[tx][ty] <= max_obstacle_distance && !in_queue[tx][ty]) {
            que.emplace(tx, ty);
            in_queue[tx][ty] = true;
          }
        }
      }
    }
  }
}

void show_curvature_graph(PathPlanner* planner) {
  constexpr int graph_cols = 600, graph_rows = 160, border = 5;
  constexpr int rows = graph_rows + 2 * border, cols = graph_cols + 2 * border;
  constexpr int num_print    = 20;
  constexpr int text_voffset = 15;

  cv::Mat view = cv::Mat::zeros(rows, cols, CV_8UC3);
  view         = cv::Scalar(255, 255, 255);

  // draw axises
  cv::line(view, {border, rows - border}, {border, border},
           cv::Scalar(0, 0, 0));
  cv::line(view, {border, rows - border}, {cols - border, rows - border},
           cv::Scalar(0, 0, 0));
  cv::line(view, {border, rows - border - graph_rows / 2},
           {cols - border, rows - border - graph_rows / 2},
           cv::Scalar(200, 200, 200), 1, cv::LineTypes::LINE_4);

  // draw curvatures
  auto res = vector<SpeedPath>();
  planner->getResults(res);
  for (auto& path : res) {
    int    idx     = 0;
    double total_s = path.path.back().s;
    double max_k   = 0.0;
    for (auto& p : path.path) max_k = max(max_k, fabs(p.k));
    max_k *= 2.0;
    max_k = min(max_k, 0.2);

    for (auto& p : path.path) {
      double rel_s = p.s / total_s;
      int    c     = border + (int)round(rel_s * graph_cols);
      double rel_k = min(p.k, max_k);
      rel_k        = max(p.k, -max_k);
      rel_k /= max_k;
      int r =
          rows - border - graph_rows / 2 - (int)round(rel_k * graph_rows / 2);
      view.at<cv::Vec3b>(r, c) = {255 * (int)p.backward, 0,
                                  255 * (int)!p.backward};
      int idx_mod              = ((++idx) % (num_print * 2));
      if (idx_mod == 0 || idx_mod == num_print) {
        cv::Scalar color = {idx_mod ? 200.0 : 0.0, idx_mod ? 0.0 : 200.0, 0};
        cv::circle(view, Point(c, r), 2, color);
        string        text      = to_string(p.k).substr(0, 6);
        constexpr int font_face = FONT_HERSHEY_PLAIN;
        auto          size = cv::getTextSize(text, font_face, 1.0, 1.0, NULL);
        cv::putText(view, text,
                    Point(c - size.width / 2,
                          r + (idx_mod == 0 ? -text_voffset
                                            : text_voffset + size.height)),
                    font_face, 0.8, color);
      }
    }
  }

  cv::imshow("Path Curvatures", view);
}

void draw_planner_map(PathPlanner* planner, cv::Mat& view) {
  // draw results
  auto res = vector<SpeedPath>();
  planner->getResults(res);
  int t = 0;
  for (auto& path : res) {
    for (auto& p : path.path) {
      int i = mtov(lround(p.x));
      int j = mtov(lround(p.y));
      if (i >= 0 && i < view.rows && j >= 0 && j < view.cols) {
        view.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 100, 0);
        t                        = ++t;
        if (t % 4 == 0 || t == 0 || t >= path.path.size()) {
          Point2f vertices[4];
          double  d =
              (CAR_LENGTH / 2 - CAR_FRONT_AXLE_TO_HEAD) / GRID_RESOLUTION;
          double new_x = p.x - d * cos(p.ang);
          double new_y = p.y - d * sin(p.ang);
          cv::RotatedRect(Point2f(mtov(new_y), mtov(new_x)),
                          Size2f(mtov(CAR_LENGTH / GRID_RESOLUTION),
                                 mtov(CAR_WIDTH / GRID_RESOLUTION)),
                          (M_PI_2 - p.ang) * 180.0 / M_PI)
              .points(vertices);
          for (int k = 0; k < 4; ++k) {
            int l = t * 240 / path.path.size();
            line(view, vertices[k], vertices[(k + 1) % 4], Scalar(0, 150, 50));
          }
        }
      }
    }
  }
  // draw target pose
  cv::arrowedLine(view, mtov(cv::Point(targetPose.y, targetPose.x)),
                  mtov(cv::Point(targetPose.y + 20 * sin(targetPose.ang),
                                 targetPose.x + 20 * cos(targetPose.ang))),
                  cv::Scalar(0, 200, 0), 2, 8, 0, 0.3);
  // draw start pose
  cv::arrowedLine(view, mtov(cv::Point(CAR_CEN_COL, CAR_CEN_ROW)),
                  mtov(cv::Point(CAR_CEN_COL, CAR_CEN_ROW - 20)),
                  cv::Scalar(0, 0, 200), 2, 8, 0, 0.3);
}

void show_heuristic(PathPlanner* planner, double safe_map[MAX_ROW][MAX_COL]) {
  pair<double, double> xya_distance[MAX_ROW][MAX_COL];
  planner->getDistanceMaps(xya_distance);
  double max_dis = 1e-5;
  for (int i = 0; i < MAX_ROW; ++i)
    for (int j = 0; j < MAX_COL; ++j) {
      if (xya_distance[i][j].first > 10000.0 || safe_map[i][j] == 0.0)
        xya_distance[i][j].first = -1.0;
      max_dis = max(max_dis, xya_distance[i][j].first);
    }
  cv::Mat view_xya = cv::Mat::zeros(MAX_ROW, MAX_COL, CV_8UC3);
  for (int i = 0; i < MAX_ROW; ++i)
    for (int j = 0; j < MAX_COL; ++j) {
      double f_xya = xya_distance[i][j].first / max_dis;
      if (f_xya >= 0.0)
        view_xya.at<Vec3b>(i, j) = Vec3b(0, (1 - f_xya) * 230, 255);
    }
  for (int i = 0; i <= MAX_ROW; i += 12)
    for (int j = 0; j <= MAX_COL; j += 12) {
      if (xya_distance[i][j].first < 0.0) continue;
      double start_x = j + 0.0;
      double start_y = i + 0.0;
      double theta   = xya_distance[i][j].second;
      double end_x   = start_x + sin(theta) * 8.0;
      double end_y   = start_y + cos(theta) * 8.0;
      cv::arrowedLine(view_xya, {(int)start_x, (int)start_y},
                      {(int)end_x, (int)end_y}, cv::Scalar(0, 0, 128), 1, 8, 0,
                      0.3);
    }
  cv::imshow("Heuristic Values", view_xya);
}

void test_steering_functions(double x, double y, double a) {
  State          start = State{CAR_CEN_ROW, CAR_CEN_COL, M_PI, 0.0, 1.0};
  State          end   = State{x, y, a, 0.0, 1.0};
  static cv::Mat view  = cv::Mat::zeros(MAX_ROW, MAX_COL, CV_8UC3);
  view *= 0;

  CC00_Dubins_State_Space dubins_space(1.0 / 25.0, 0.001, 1.0);
  auto                    controls = dubins_space.get_controls(start, end);
  for (double i = 0.0; i <= 1.0; i += 0.01) {
    State stat = dubins_space.interpolate(start, controls, i);
    view.at<cv::Vec3b>(lround(stat.x), lround(stat.y)) = {255, 255, 255};
  }

  HC_Reeds_Shepp_State_Space rs_space(1.0 / 25.0, 0.001, 1.0);
  controls = rs_space.get_controls(start, end);
  for (double i = 0.0; i <= 1.0; i += 0.01) {
    State stat = rs_space.interpolate(start, controls, i);
    view.at<cv::Vec3b>(lround(stat.x), lround(stat.y)) = {255, 0, 255};
  }

  cv::imshow("CC Path Test", view);
}

string rand_string(int length) {
  const char characs[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  string     buffer;

  random_device         rd;
  default_random_engine random(rd());

  for (int i = 0; i < length; i++)
    buffer += characs[random() % sizeof(characs)];
  return buffer;
}

int main(int argc, char** argv) {
#ifdef DEBUG_EXPANSION_CALLBACK
  node_expanded = on_node_expanded;
#endif
#ifdef DEBUG_ANALYTIC_EXPANSION_CALLBACK
  analytic_expanded = on_analytic_expanded;
#endif
  read_args(argc, argv);
  PathPlanner* planner = PathPlanner::getInstance();
  cv::namedWindow("PathPlanner Test");
  if (!target_preset) cv::setMouseCallback("PathPlanner Test", on_mouse);
  cv::Mat map_image = cv::imread(map_file_path);
  cv::resize(map_image, map_image, cv::Size(MAX_COL, MAX_ROW));
  double safe_map[MAX_ROW][MAX_COL] = {0};
  for (int i = 0; i < MAX_ROW; ++i)
    for (int j = 0; j < MAX_COL; ++j)
      if (map_image.at<cv::Vec3b>(i, j)[0] < 30 &&
          map_image.at<cv::Vec3b>(i, j)[1] < 30 &&
          map_image.at<cv::Vec3b>(i, j)[2] < 30)
        safe_map[i][j] = 0;
      else
        safe_map[i][j] = 1e8;
  generate_safe_map(safe_map);
  map_view = cv::Mat(mtov(MAX_ROW), mtov(MAX_COL), CV_8UC3, {255, 255, 255});
  for (int i = 0; i < map_view.rows; ++i)
    for (int j = 0; j < map_view.cols; ++j) {
      double safe_value = safe_map[(int)vtom(i)][(int)vtom(j)];
      if (safe_value == 0.0)
        map_view.at<cv::Vec3b>(i, j) = Vec3b(0, 0, 0);
      else if (safe_value <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION)
        map_view.at<cv::Vec3b>(i, j) = cv::Vec3b(200, 245, 255);
    }

  cv::imshow("PathPlanner Test", map_view);
  do {
    if (!target_preset) {
      while (operationStatus != Planning) cv::waitKey(30);
    }

    analytic_expansion_layer = (map_view.clone() = 0.0);
    expansion_layer          = (map_view.clone() = 0.0);

    string key = rand_string(5);

    log_0("[", key, "] ", "target ", targetPose.x, " ", targetPose.y, " ",
          targetPose.ang);
    cerr << "[" << key << "] planner start" << endl;
    planner->setBackwardEnabled(backward_enabled);
    planner->setCurrentSpeed(current_speed);
    planner->setStartMaintainedPath(vector<TiEV::Pose>());
    planner->setAbsSafeMap(safe_map);
    planner->setLaneSafeMap(safe_map);
    planner->setTargets(vector<TiEV::Pose>(1, targetPose));
    planner->plan();

    cv::Mat main_view = map_view.clone();
    merge_layer(main_view, expansion_layer, 0.4);
    draw_planner_map(planner, main_view);
    cv::imshow("PathPlanner Test", main_view);
    show_curvature_graph(planner);
    show_heuristic(planner, safe_map);
    if (target_preset)
      cv::waitKey(0);
    else
      operationStatus = WaitingForTargetPoint;
  } while (!target_preset);
  return 0;
}