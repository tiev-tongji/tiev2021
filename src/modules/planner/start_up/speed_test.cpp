#include "speed_optimizer.h"
#include <fstream>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "speed_view/speed_view.h"
#include <unistd.h>
#include <thread>
#include "time_counter/time_counter.h"

using namespace TiEV;
using namespace SplineLib;
using namespace std;

int main() {
    ifstream obstacle_log_file("/home/autolab/obstacle_log_file.txt");
    ifstream path_log_file("/home/autolab/path_log_file.txt");
    ifstream speed_limit_log_file("/home/autolab/speed_limit_log_file.txt");

    if (!obstacle_log_file.is_open() || !path_log_file.is_open() || !speed_limit_log_file.is_open()) {
        cout << "Fail to open log files!" << endl;
        return 0;
    }

    // Read in obstacle info from log file
    vector<Obstacle> obstacle_list;
# if 0
    size_t obj_num;
    obstacle_log_file >> obj_num;
    for (size_t i = 0; i < obj_num; ++i) {
        Obstacle obj;
        obstacle_log_file >> obj.width >> obj.length;
        size_t point_num;
        obstacle_log_file >> point_num;
        for (size_t j = 0; j < point_num; ++j) {
            Point point;
            obstacle_log_file >> point.x >> point.y;
            double rad; obstacle_log_file >> rad;
            point.angle.setByRad(rad);
            obstacle_log_file >> point.v >> point.t;
            obj.path.push_back(point);
        }
        obstacle_list.push_back(obj);
    }
#endif
    // Read in path info from log file
    vector<Point> path;
    size_t point_num;
    path_log_file >> point_num;
    for (size_t i = 0; i < point_num; ++i) {
        Point point;
        path_log_file >> point.x >> point.y;
        double rad; path_log_file >> rad;
        point.angle.setByRad(rad);
        path_log_file >> point.s >> point.v;
        path.push_back(point);
    }

    // Read in speed limit info from log file
    vector<pair<double, double> > speed_limit;
    size_t limit_num;
    speed_limit_log_file >> limit_num;
    for (size_t i = 0; i < limit_num; ++i) {
        double s, v;
        speed_limit_log_file >> s >> v;
        speed_limit.push_back(make_pair(s, v));
    }

    SpeedOptimizer speed_optimizer(obstacle_list, path, speed_limit,
                                   0, path.back().s, 0, 10);

    // manually add some st_boundaries
    vector<STBoundary> st_boundaries_for_test;
    STBoundary st_boundary1(STPoint(14.3039, 0), STPoint(14.3039, 10), STPoint(14.7561, 0), STPoint(14.7561, 10));
    st_boundaries_for_test.emplace_back(st_boundary1);
    speed_optimizer.SetStBoundaries(st_boundaries_for_test);

    // Debug
    cout << "SpeedOptimizer initialization succeed" << endl;
    cout << "obstacle info: " << endl;
    for (const auto& obj : obstacle_list) {
        cout << " width && length: " << obj.width << ' ' << obj.length << endl;
        cout << "obstacle path: " << endl;
        for (const auto& p : obj.path) {
            cout << " x y theta v t: " << p.x << ' ' << p.y << ' ' << p.angle.getRad() << ' ' << p.v << ' ' << p.t << endl;
        }
    }

    if (speed_optimizer.DP_Process()) {
        cout << "DP good" << endl;

        cout << "DP speed profile: " << endl;
        for (const auto& p : speed_optimizer.dp_speed_data()) {
            cout << p.s() << ' ' << p.t() << endl;
        }
    } else {
        cout << "DP bad" << endl;
        return 0;
    }

    if (speed_optimizer.QP_Process()) {
        cout << "QP good" << endl;
    } else {
        cout << "QP bad" << endl;
    }


#if 0
	Visualization *vs = Visualization::getInstance();
	thread visualization = thread(&Visualization::visualize, vs);
	visualization.detach();

	SpeedView *sv = SpeedView::getInstance();
	vector<Coefficient> coes;
    const Spline1d splines = speed_optimizer.splines();
    for (const auto& spline_seg : splines.splines()) {
        const PolynomialXd& poly = spline_seg.spline_func();
        coes.emplace_back(Coefficient(poly.params()));
        cout << endl;
    }

	int test1 = 0;
	int test2 = 0;
	int test3 = 0;
	while(true){
		test1++;
		test2 += 2;
		test3 += 3;
		vs->print_text("test1", test1);
		vs->print_text("test2", test2);
		vs->print_text("test3", test3);
		vs->print_text("a_begin", "value");
		vs->print_text("x_begin", "value");
		// sv->setSplines(splines, splines_num);
		sv->setCoefficients(coes);
		sv->setSTBoundaries(st_boundaries_for_test);
		sv->draw();
		usleep(200*1000);
	}
#endif
	return 0;
}

