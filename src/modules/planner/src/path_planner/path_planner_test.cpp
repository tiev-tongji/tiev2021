#include "path_planner/path_planner.h"
#include "planner_common/config.h"
#include "planner_common/pose.h"
#include "planner_common/const.h"
#include <iostream>
#include <termio.h>
#include <thread>
#include "opencv2/opencv.hpp"

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
double          current_speed    = 0;

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
                current_speed = atof(argv[i++]);
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
    queue<pair<int, int>> que;
    for(int i = 0; i < MAX_ROW; ++i)
        for(int j = 0; j < MAX_COL; ++j)
            if(safe_map[i][j] == 0)
                que.push(make_pair(i, j));

    while(!que.empty()){
        int nx = que.front().first;
        int ny = que.front().second;
        que.pop();
        for(int dx = -1; dx <= 1; ++dx){
            for(int dy = -1; dy <= 1; ++dy){
                int tx = dx + nx;
                int ty = dy + ny;
                if(tx >= 0 && tx < MAX_ROW &&
                    ty >= 0 && ty < MAX_COL){
                    if(safe_map[nx][ny] + 1 < safe_map[tx][ty]){
                        safe_map[tx][ty] = safe_map[nx][ny] + 1;
                        que.push(make_pair(tx, ty));
                    }
                }
            }
        }
    }
}

void draw(PathPlanner* planner, cv::Mat& view) {
    // draw results
    auto res = vector<SpeedPath>();
    planner->getResults(res);
    for(auto& path : res) {
        for(auto& p : path.path) {
            int i = round(p.x), j = round(p.y);
            view.at<cv::Vec3b>(i, j) = cv::Vec3b(0x00, 0x00, 0xff);
        }
    }
    // draw target pose
    cv::arrowedLine(view,
        cv::Point(targetPose.y, targetPose.x),
        cv::Point(
            targetPose.y + 20 * sin(targetPose.ang),
            targetPose.x + 20 * cos(targetPose.ang)
        ),
        cv::Scalar(255, 0, 0)
    );
    // draw start pose
    cv::arrowedLine(view,
        cv::Point(CAR_CEN_COL, CAR_CEN_ROW),
        cv::Point(CAR_CEN_COL, CAR_CEN_ROW - 20),
        cv::Scalar(255, 128, 0)
    );
}

int main(int argc, char** argv){
    read_args(argc, argv);
    PathPlanner* planner = PathPlanner::getInstance();
    cv::namedWindow("PathPlanner Test", cv::WINDOW_NORMAL);
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

        cv::Mat view_image = map_image.clone();
        draw(planner, view_image);
        cv::imshow("PathPlanner Test", view_image);
        operationStatus = WaitingForTargetPoint;
    }
    return 0;
}