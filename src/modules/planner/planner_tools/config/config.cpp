#include "config.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <fstream>
#include <iostream>

using namespace rapidjson;
using namespace std;

namespace TiEV {

void Config::init(){
	string para_file_path = TiEV_CONFIG_DIRECT + "plannerPara.json";
    fstream input(para_file_path, ios::in);
    if(!input.is_open()) {
		throw "Open config failed: file not exist.";
	}
    stringstream buffer;
    Document doc;
    buffer << input.rdbuf();
    doc.Parse(buffer.str().c_str());
    #define nameof(x) (#x)
    roadmap_file = doc[nameof(roadmap_file)].GetString();
    control_mode = (ControlMode)(doc[nameof(control_mode)].GetInt());
    debug_event_mode = (RoadMode)(doc[nameof(debug_event_mode)].GetInt());
    debug_speed_mode = (SpeedMode)(doc[nameof(debug_speed_mode)].GetInt());
    targets_num_limit = doc[nameof(targets_num_limit)].Get<int>();
    plan_time_limit_ms = doc[nameof(plan_time_limit_ms)].Get<int>();
    car_away_limit_meter = doc[nameof(car_away_limit_meter)].Get<double>();
    rs_distance_table_path = doc[nameof(rs_distance_table_path)].GetString();
    dubins_distance_table_path = doc[nameof(dubins_distance_table_path)].GetString();
    a_star_extention_step_meter = doc[nameof(a_star_extention_step_meter)].Get<double>();
    a_star_analytic_expansion_param_k = doc[nameof(a_star_analytic_expansion_param_k)].Get<double>();
    a_star_analytic_expansion_param_t = doc[nameof(a_star_analytic_expansion_param_t)].Get<double>();
    a_star_analytic_expansion_max_N = doc[nameof(a_star_analytic_expansion_max_N)].Get<int>();
	tasks.clear();
	auto task_arr = doc["tasks"].GetArray();
	tasks.resize(task_arr.Size());
	for(int i = 0; i < task_arr.Size(); ++i){
        auto task_points_arr = task_arr[i]["task_points"].GetArray();
        tasks[i].task_points.resize(task_points_arr.Size());
        for(int j = 0; j < task_points_arr.Size(); ++j){
		    double lat = task_points_arr[j]["lat"].GetDouble();
		    double lon = task_points_arr[j]["lon"].GetDouble();
		    double heading = task_points_arr[j]["heading"].GetDouble();
            tasks[i].task_points[j] = Point::fromLatLon(lat, lon);
            tasks[i].task_points[j].heading.setByRad(heading);
        }
	}
    #undef nameof
    input.close();
    outputConfigures();
}

void Config::outputConfigures() const{
    cout << "--------------------" << endl << "Config"
        << endl << "--------------------" << endl;

    #define print(a) cout << #a << " = " << (a) << endl
    print(roadmap_file);
    print(rs_distance_table_path);
    print(dubins_distance_table_path);
    print(control_mode);
    print(debug_event_mode);
    print(debug_speed_mode);
    print(targets_num_limit);
    print(plan_time_limit_ms);
    print(car_away_limit_meter);
    print(a_star_extention_step_meter);
    print(a_star_analytic_expansion_param_k);
    print(a_star_analytic_expansion_param_t);
    print(a_star_analytic_expansion_max_N);
    print(tasks.size());
    for(auto& task : tasks){
        cout << "- ";
        for(auto& point : task.task_points)
            cout << "(" << point.lat << "," <<
                point.lon << "," << point.heading.getRad() << ") ";
        cout << endl;
    }
    #undef print
}

//Config Config::inner_instance;

}
