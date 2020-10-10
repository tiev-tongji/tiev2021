#include "decision_state.h"

namespace TiEV{

/*a demo state implement*/
void LaneDriving::entry(){
	cout << "Entry LaneDriving State..." << endl;
	vs->print_text("State", "LaneDriving", 1);
}

void LaneDriving::exit(){
	cout << "Exit LaneDriving State..." << endl;
}

void LaneDriving::react(PlanningEvent const & e){
	if(getInfoByNavinfo(e.nav_info) || (!e.lanes.detected)) {
		transit<NormalDriving>();
	}

	if(e.slam_info.reliable){
		transit<Parking>();
		return;
	}

    getMap(e.lidar_map, e.lanes);
	vector<Point> target_points = getTargetsOfLanes(e.lanes);
    if (!plan(e.dynamic_objs, getSpeedByMode(4), false)) {
        speedReplan(e.dynamic_objs);
    }
    updateMaintainedPath(e.nav_info, best_path.path);
	visualization(e);
}

}//end TiEV
