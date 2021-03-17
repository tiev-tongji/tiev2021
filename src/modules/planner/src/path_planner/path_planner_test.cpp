#include "opencv2/opencv.hpp"
#include "path_planner/path_planner.h"
#include "planner_common/config.h"
#include "planner_common/const.h"
#include "planner_common/pose.h"
#include "planner_common/collision_check.h"
#include <iostream>
#include <termio.h>
#include <thread>

using namespace std;
using namespace TiEV;
using namespace cv;

enum OperationStatus {
    WaitingForTargetPoint,
    WaitingForTargetAngle,
    Planning,
};

OperationStatus operationStatus = WaitingForTargetPoint;
TiEV::Pose      targetPose;
char*           map_file_path = NULL;
bool            backward_enabled = false;
double          current_speed = 0;

void usage() {
    exit(0);
}

void read_args(int argc, char** argv) {
    for(int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--enable-backward") == 0) {
            cout << "backward enabled" << endl;
            backward_enabled = true;
        }
        else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--speed") == 0) {
            if (i + 1 < argc) {
                current_speed = atof(argv[++i]);
                cout << "current speed set: " << current_speed << " m/s" << endl;
            }
            else {
                cout << "error using flag " << argv[i] << " , argument missing." << endl;
                usage();
            }
        }
        else if (map_file_path == NULL) {
            map_file_path = argv[i];
            cout << "map file path: " << map_file_path << endl;
        }
        else {
            cout << "unrecognized input " << argv[i] << endl;
            usage();
        }
    }
}

void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	switch (operationStatus) {
        case WaitingForTargetPoint: {
            if (event == CV_EVENT_LBUTTONDOWN) {
                targetPose.x = y;
                targetPose.y = x;
                operationStatus = WaitingForTargetAngle;
            }
            break;
        }
        case WaitingForTargetAngle: {
            if (event == CV_EVENT_LBUTTONUP) {
                targetPose.ang = atan2(x - targetPose.y, y - targetPose.x);
                operationStatus = Planning;
            }
            break;
        }
    }
}

void generate_safe_map(double safe_map[MAX_ROW][MAX_COL]) {
    for (int row = 0; row < MAX_ROW; ++row)
        for (int column = 0; column < MAX_COL; ++column) {
            if (safe_map[row][column] == 0.0) continue;
            int start_r = (row == 0 || column == 0) ?
                0 : max(max(safe_map[row - 1][column] - 1, 0.0),
                max(safe_map[row][column - 1] - 1, 0.0)) / sqrt(2);
            int end_r = min(50, max(max(row + 1, MAX_ROW - row),
                max(column + 1, MAX_COL - column)));
            double now_min_dis = end_r;
            for (int r = start_r; r < end_r; ++r) {
                int xs[] = { row - r, row - r, row + r, row + r };
                int ys[] = { column - r, column + r, column + r, column - r };
                constexpr int dxs[] = { 0, 1, 0, -1 };
                constexpr int dys[] = { 1, 0, -1, 0 };
                for (int i = 0; i < 2 * r; ++i) {
                    for (int d = 0; d < 4; ++d) {
                        if (xs[d] >= 0 && xs[d] < MAX_ROW &&
                            ys[d] >= 0 && ys[d] < MAX_COL &&
                            safe_map[xs[d]][ys[d]] == 0) {
                            double dis = sqrt(
                                (xs[d] - row) * (xs[d] - row) +
                                (ys[d] - column) * (ys[d] - column));
                            if (dis < now_min_dis) {
                                now_min_dis = dis;
                                end_r = ceil(now_min_dis);
                            }
                        }
                        xs[d] += dxs[d];
                        ys[d] += dys[d];
                    }
                }
            }
            safe_map[row][column] = now_min_dis;
        }
}

void show_curvature_graph(PathPlanner* planner) {
    int graph_cols = 600, graph_rows = 160, border = 20;
    int rows = graph_rows + 2 * border,
        cols = graph_cols + 2 * border;
    cv::Mat view = cv::Mat::zeros(rows, cols, CV_8UC3);
    view = cv::Scalar(255, 255, 255);

    // draw axises
    cv::line(view, {border, rows - border}, {border, border}, cv::Scalar(0, 0, 0));
    cv::line(view, {border, rows - border}, {cols - border, rows - border}, cv::Scalar(0, 0, 0));
    cv::line(view, {border, rows - border - graph_rows / 2},
        {cols - border, rows - border - graph_rows / 2},
        cv::Scalar(50, 50, 50), 1, cv::LineTypes::LINE_4);

    // draw curvatures
    auto res = vector<SpeedPath>();
    planner->getResults(res);
    for(auto& path : res) {
        double total_s = path.path.back().s;
        for(auto& p : path.path) {
            double rel_s = p.s / total_s;
            int c = border + (int)round(rel_s * graph_cols);
            double rel_k = min(p.k, 0.3);
            rel_k = max(p.k, -0.3);
            rel_k /= 0.3;
            int r = rows - border - graph_rows / 2 - (int)round(rel_k * graph_rows / 2);
            view.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);
        }
    }

    cv::imshow("Path Curvatures", view);
}

void draw_planner_map(PathPlanner* planner, cv::Mat& view) {
    int costs[MAX_ROW][MAX_COL] = {0};
    planner->getCostMap(costs);
    for (int i = 0; i < MAX_ROW; ++i)
        for (int j = 0; j < MAX_COL; ++j) {
            if (costs[i][j] == 0) continue;
            double rel_cost = min(costs[i][j] / 50.0, 1.0);
            view.at<cv::Vec3b>(i, j) = cv::Vec3b(
                100 * (1 - rel_cost) + 155,
                100 * (1 - rel_cost) + 155, 255);
        }

    // draw results
    auto res = vector<SpeedPath>();
    planner->getResults(res);
    int t = 0;
    for(auto& path : res) {
        for(auto& p : path.path) {
            int i = round(p.x), j = round(p.y);
            view.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 100, 0);
            t = (t + 1) % 10;
            if (t == 0) {
                Point2f vertices[4];
                cv::RotatedRect(
                    Point2f(p.y, p.x),
                    Size2f(CAR_LENGTH / GRID_RESOLUTION,
                        CAR_WIDTH / GRID_RESOLUTION),
                    (M_PI_2 - p.ang) * 180.0 / M_PI).points(vertices);
                for (int k = 0; k < 4; ++k)
                    line(view, vertices[k], vertices[(k + 1) % 4],
                        Scalar(128, 200, 0));
            }
        }
    }
    // draw target pose
    cv::arrowedLine(view,
        cv::Point(targetPose.y, targetPose.x),
        cv::Point(targetPose.y + 20 * sin(targetPose.ang),
            targetPose.x + 20 * cos(targetPose.ang)),
        cv::Scalar(0, 0, 0), 2, 8, 0, 0.3);
    // draw start pose
    cv::arrowedLine(view,
        cv::Point(CAR_CEN_COL, CAR_CEN_ROW),
        cv::Point(CAR_CEN_COL, CAR_CEN_ROW - 20),
        cv::Scalar(0, 0, 0), 2, 8, 0, 0.3);
}

void show_heuristic(PathPlanner* planner) {
    double xy_distance[MAX_ROW][MAX_COL];
    pair<double, double> xya_distance[MAX_ROW][MAX_COL];
    planner->getDistanceMaps(xy_distance, xya_distance);
    double max_dis = 1e-5;
    for (int i = 0; i < MAX_ROW; ++i)
        for (int j = 0; j < MAX_COL; ++j) {
            if (xy_distance[i][j] > 10000.0) xy_distance[i][j] = -1.0;
            if (xya_distance[i][j].first > 10000.0)
                xya_distance[i][j].first = -1.0;
            max_dis = max(max_dis, xy_distance[i][j]);
            max_dis = max(max_dis, xya_distance[i][j].first);
        }
    cv::Mat view_xy = cv::Mat::zeros(MAX_ROW, MAX_COL, CV_8UC3);
    cv::Mat view_xya = cv::Mat::zeros(MAX_ROW, MAX_COL, CV_8UC3);
    for (int i = 0; i < MAX_ROW; ++i)
        for (int j = 0; j < MAX_COL; ++j) {
            double f_xy = xy_distance[i][j] / max_dis;
            double f_xya = xya_distance[i][j].first / max_dis;
            if (f_xy >= 0.0) view_xy.at<Vec3b>(i, j) =
                Vec3b(0, (1 - f_xy) * 200, 255);
            if (f_xya >= 0.0) view_xya.at<Vec3b>(i, j) =
                Vec3b(0, (1 - f_xya) * 230, 255);
            if (((int)round(f_xy * 500)) % 10 == 0)
                view_xy.at<Vec3b>(i, j) = Vec3b(0, 0, 128);
        }
    for (int i = 0; i <= MAX_ROW; i += 12)
        for (int j = 0; j <= MAX_COL; j += 12) {
            if (xya_distance[i][j].first < 0.0) continue;
            double start_x = j + 0.0;
            double start_y = i + 0.0;
            double theta = xya_distance[i][j].second;
            double end_x = start_x + sin(theta) * 8.0;
            double end_y = start_y + cos(theta) * 8.0;
            cv::arrowedLine(view_xya,
                {(int)start_x, (int)start_y},
                {(int)end_x, (int)end_y},
                cv::Scalar(0, 0, 128), 1, 8, 0, 0.3);
        }
    cv::Mat view = Mat(MAX_ROW, 2 * MAX_COL, CV_8UC3);
    view_xy.copyTo(view(Rect(0, 0, MAX_COL, MAX_ROW)));
    view_xya.copyTo(view(Rect(MAX_COL, 0, MAX_COL, MAX_ROW)));
    cv::imshow("Heuristic Values", view);
}

int main(int argc, char** argv){
    read_args(argc, argv);
    PathPlanner* planner = PathPlanner::getInstance();
    cv::namedWindow("PathPlanner Test");
    cv::setMouseCallback("PathPlanner Test", on_mouse);
    cv::Mat map_image = cv::imread(map_file_path);
    cv::resize(map_image, map_image, cv::Size(MAX_COL, MAX_ROW));
    double safe_map[MAX_ROW][MAX_COL] = {0};
    for(int i = 0; i < MAX_ROW; ++i)
        for(int j = 0; j < MAX_COL; ++j)
            if(map_image.at<cv::Vec3b>(i, j)[0] < 200)
                safe_map[i][j] = 0;
            else safe_map[i][j] = 1e8;
    generate_safe_map(safe_map);
    cv::imshow("PathPlanner Test", map_image);
    while (true) {
        while (operationStatus != Planning) cv::waitKey(30);

        planner->setBackwardEnabled(backward_enabled);
        planner->setCurrentSpeed(current_speed);
        planner->setStartMaintainedPath(vector<TiEV::Pose>());
        planner->setAbsSafeMap(safe_map);
        planner->setLaneSafeMap(safe_map);
        planner->setTargets(vector<TiEV::Pose>(1, targetPose));
        planner->plan();

        cv::Mat view_image = cv::Mat::zeros(MAX_ROW, MAX_COL, CV_8UC3);
        view_image = cv::Scalar(255, 255, 255);
        for(int i = 0; i < MAX_ROW; ++i)
            for(int j = 0; j < MAX_COL; ++j) {
                if (safe_map[i][j] == 0.0)
                    view_image.at<cv::Vec3b>(i, j) = Vec3b(0, 0, 0);
                else if (safe_map[i][j] <= COLLISION_CIRCLE_SMALL_R / GRID_RESOLUTION)
                    view_image.at<cv::Vec3b>(i, j) = Vec3b(200, 245, 255);
            }
        draw_planner_map(planner, view_image);
        cv::imshow("PathPlanner Test", view_image);
        show_curvature_graph(planner);
        show_heuristic(planner);
        operationStatus = WaitingForTargetPoint;
    }
    return 0;
}