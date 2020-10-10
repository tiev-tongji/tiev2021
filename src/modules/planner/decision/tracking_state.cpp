#include "decision_state.h"

namespace TiEV{

/*a demo state implement*/
void Tracking::entry(){
	cout << "Entry Tracking State..." << endl;
	vs->print_text("State", "Tracking", 1);
}

void Tracking::exit(){
	cout << "Exit Tracking State..." << endl;
}

void Tracking::react(PlanningEvent const & e){
/*
	if(!e.nav_info.reliable){
		transit<NormalDriving>();
		return;
	}
*/

	getInfoByNavinfo(e.nav_info);
	if(completTask()){
		transit<StopState>();
		return;
	}

/*
	if(RoadMode::TRACKING_MODE != road_mode){
		transit<NormalDriving>();
		return;
	}
*/

	getMap(e.lidar_map, e.lanes);
    maintained_path = reference_path;
	speedReplan(e.dynamic_objs);
	updateMaintainedPath(e.nav_info, best_path.path);
	visualization(e);
}

}//end TiEV
