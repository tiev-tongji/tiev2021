#include "decision_state.h"

namespace TiEV{

void StopState::entry(){
	cout << "Entry StopState State..." << endl;
	stop_lock.lock(8*1000);
	vs->print_text("State", "StopState", 1);
}

void StopState::exit(){
	cout << "Exit StopState State..." << endl;
}

void StopState::react(PlanningEvent const & e){
//----------debug-------
	//transit<NormalDriving>();
	//return;
//----------------------
/*
	vs->print_text("Stop to GlobalPlan", stop_lock.isLock());
	if(!stop_lock.isLock()){
		updateMaintainedPath(e.nav_info, reference_path);
		transit<GlobalPlanning>();
		return;
	}
	*/
	vector<Point> stop_path;
	for(int i = 0; i < 10; i++){
		Point p;
		p.x  = 300 - i;
		p.y = 75;
		p.angle.setByRad(0);
		p.v = 0;
		p.a = 0;
		p.k = 0;
		p.s = i * GRID_RESOLUTION;
		p.t = i;;
		stop_path.push_back(p);
	}
	updateMaintainedPath(e.nav_info, stop_path);
}

}//end TiEV
