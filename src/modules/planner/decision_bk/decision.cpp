#include "decision.h"
#include <unistd.h>

/*initial fsm*/
FSM_INITIAL_STATE(TiEV::DecisionFSM, TiEV::NormalDriving)

using namespace std;

namespace TiEV{

/*run decision FSM*/
void Decision::runDecision(){
	//decision fsm start
	dc::start();
	Visualization *vs = Visualization::getInstance(); while(true){
		long int time_begin = getTimeStamp();
		//demo: decision fsm execute planning event
		planning_event.refresh();
		dc::dispatch(planning_event);
		if(getTimeStamp() - time_begin < 30 * 1000){
			usleep(30*1000 - (getTimeStamp() - time_begin));
		}
		long int time_end = getTimeStamp();
		int run_time = time_end - time_begin;
		run_time /= 1000;
		vs->print_text("running time(ms)", run_time);
		if(vs->TiEV_Stop) return;
    }
	cout << "Exit decision..." << endl;
}

/*this thread send the path be executed by PIDContoller*/

void Decision::sendPidPath(){
	RoadManager *road_manager = RoadManager::getInstance();
	MessageManager *msg_manager = MessageManager::getInstance();
	vector<Point> maintained_path;
	structAIMPATH pid_path;
	NavInfo nav_info;
	while(true){
		maintained_path.clear();
		pid_path.points.clear();
		msg_manager->getNavInfo(nav_info);
		road_manager->getMaintainedPath(nav_info, maintained_path);
		for(int i = 0; i < maintained_path.size(); i++){
            if(maintained_path[i].backward != maintained_path.front().backward) break;
			TrajectoryPoint tra_point;
			tra_point.x = (CAR_CEN_ROW - maintained_path[i].x) * GRID_RESOLUTION;
			tra_point.y = (CAR_CEN_COL - maintained_path[i].y) * GRID_RESOLUTION;
			tra_point.theta = -maintained_path[i].angle.getRad();
			tra_point.t = maintained_path[i].t;
			tra_point.k = maintained_path[i].k;

            if(maintained_path[i].v < -2) tra_point.v = -2;
            else if(maintained_path[i].v > -2 && maintained_path[i].v <= -0.01) tra_point.v = -2;
            else if(maintained_path[i].v > -0.01 && maintained_path[i].v < 0.01) tra_point.v = 0;
            else if(maintained_path[i].v >= 0.1 && maintained_path[i].v < 2) tra_point.v = 2;
		    else tra_point.v = maintained_path[i].v;

			tra_point.a = maintained_path[i].a;
            //cout << "!!!!!!!!path s v angle:" << maintained_path[i].s << " " << tra_point.v << " " << tra_point.theta << endl;
		    //tra_point.v += 5;
			pid_path.points.push_back(tra_point);
		}
        if(pid_path.points.size() > 1){
            pid_path.points.back().v = pid_path.points[pid_path.points.size()-2].v;
            pid_path.points.front().v = pid_path.points[1].v;
        }
		pid_path.num_points = pid_path.points.size();
		//for no pid path to control
		if(pid_path.num_points <= 0) setStopPidPath(pid_path);
		msg_manager->publishPath(pid_path);
        usleep(10*1000);
	}
}

void Decision::setStopPidPath(structAIMPATH &_pid_path){
	if(_pid_path.points.empty()){
		for(int i = 0; i < 10; i++){
			TrajectoryPoint tra_point;
			tra_point.x = i * GRID_RESOLUTION;
			tra_point.y = 0;
			tra_point.theta = 0;
			tra_point.t = i;
			tra_point.k = 0;
			tra_point.v = 0;
			tra_point.a = 0;
			_pid_path.points.push_back(tra_point);
		}
		_pid_path.num_points = _pid_path.points.size();
	}else{
		for(auto& p: _pid_path.points) p.v = 0;
	}
}

}
