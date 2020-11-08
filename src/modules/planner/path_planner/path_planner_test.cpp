#include "path_planner.h"
#include "config/config.h"
#include <iostream>
#include "nature.h"
#include <termio.h>
#include <thread>
#include "nature/point.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace TiEV;
using namespace cv;

#ifdef NO_SPEED_PLANNER

int mmp[GRID_ROW][GRID_COL] = {0};

vector<TiEV::Point> res[MAX_TARGET_NUM];

int getch(void) {
    int cr;
    struct termios nts, ots;
    if (tcgetattr(0, &ots) < 0) return EOF;
    nts = ots;
    cfmakeraw(&nts);
    if (tcsetattr(0, TCSANOW, &nts) < 0) return EOF;
    cr = getchar();
    if (tcsetattr(0, TCSANOW, &ots) < 0) return EOF;
    return cr;
}

void setColor(cv::Mat& mat, int row, int col, u_char r, u_char g, u_char b){
    mat.at<cv::Vec3b>(row, col)[0] = b;
    mat.at<cv::Vec3b>(row, col)[1] = g;
    mat.at<cv::Vec3b>(row, col)[2] = r;
}

int main(){
    Config* config = Config::getInstance();
    PathPlanner* planner = PathPlanner::getInstance();
    PathPlannerView* plannerView = PathPlannerView::getInstance();

    vector<TiEV::Point> targets;
    planner->setStartMaintainedPath(vector<TiEV::Point>());
    planner->setBackwardEnabled(true);

    cv::namedWindow("PathPlanner Test", cv::WINDOW_KEEPRATIO);

    double ang;
    cout << "Target number: ";
    int n;
    while(cin >> n){
        targets.resize(n);
        for(int i = 0; i < n; ++i){
            cout << "Set StartPoint(x,y,a): ";
            cin >> targets[i].x >> targets[i].y >> ang;
            targets[i].angle.setByRad(ang);
        }

        cv::Mat view = cv::Mat::zeros(TiEV::GRID_ROW, TIEV::GRID_COL, CV_8UC3);
        cv::Mat cvmmp = cv::imread("/home/autolab/IV_planner/IV_path_exp/maps/map-09.png");
        cv::resize(cvmmp, cvmmp, cv::Size(151, 401));

		cout << "Set speed in km/h: ";
		double spd;
		cin >> spd;
		spd /= 3.6;

        memset(mmp, 0x7f, sizeof(mmp));

        queue<pair<int, int>> que;
        for(int i = 0; i < 401; ++i)
            for(int j = 0; j < 151; ++j)
                if(cvmmp.at<cv::Vec3b>(i, j)[0] < 200){
                    mmp[i][j] = 0;
                    que.push(make_pair(i, j));
                }

        while(!que.empty()){
            int nx = que.front().first;
            int ny = que.front().second;
            que.pop();
            for(int dx = -1; dx <= 1; ++dx)
                for(int dy = -1; dy <= 1; ++dy){
                    int tx = dx + nx;
                    int ty = dy + ny;
                    if(tx >= 0 && tx < GRID_ROW &&
                        ty >= 0 && ty < GRID_COL){
                        if(mmp[nx][ny] + 1 < mmp[tx][ty]){
                            mmp[tx][ty] = mmp[nx][ny] + 1;
                            que.push(make_pair(tx, ty));
                        }
                    }
                }
        }

        planner->setBackwardEnabled(false);
        planner->setCurrentSpeed(spd);
        planner->setStartMaintainedPath(vector<TiEV::Point>());
        planner->setAbsSafeMap(mmp);
        planner->setLaneSafeMap(mmp);
        targets[0].angle.setByRad(ang);
        planner->setTargets(targets);
        time_t start_time = getTimeStamp();
        planner->plan();

        cout << getTimeStamp() - start_time << "us" << endl;

        plannerView->draw(view);

        cv::imshow("PathPlanner Test", view);
        while(cv::waitKey(100)!=27);

#ifdef DEBUG
        int current_index = -1;
        char a;
        cout << "[E]: Exit point viewer, [N]: Next point, [P] Previous point" << endl;
        while((a = getch()) != 'E' && a != 'e'){
            if(current_index >= 0)
                setColor(view, round(res[0][current_index].x),
                    round(res[0][current_index].y),
                    255, 255, 255);
            if(a == 'n' || a == 'N') ++current_index;
            else if(a == 'p' || a == 'P') --current_index;
            else continue;

            while(current_index >= res[0].size()) current_index -= res[0].size();
            while(current_index < 0) current_index += res[0].size();

            setColor(view, round(res[0][current_index].x),
                round(res[0][current_index].y),
                255, 0, 0);

            cout << "\r                                                                   \rx:" << res[0][current_index].x << " y:"
                << res[0][current_index].y << " angle:"
                << res[0][current_index].angle.getRad() << " heuristic:"
                << res[0][current_index].s << " cost:"
                << res[0][current_index].v;

            cv::imshow("PathPlanner Test", view);
            waitKey(1);
        }
#endif

        cout << endl << "Target number: ";
    }

    return 0;
}

#else
int main(){
	cout << "NO_SPEED_PLANNER must be defined to test path_planner independently" << endl;
	return 0;
}
#endif
