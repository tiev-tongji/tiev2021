#include "decision_state.h"
#include "visualization.h"

namespace TiEV{

/*Free Driving state*/
void MapFree::entry() {
	cout << "Entry MapFree State..." << endl;
	vs->print_text("State", "MapFree", 1);
}

void MapFree::exit() {
	cout << "Exit MapFree State..." << endl;
}

void MapFree::react(PlanningEvent const & e)  {
	if(e.nav_info.reliable || e.lanes.detected){
		transit<NormalDriving>();
	}

	if(e.slam_info.reliable){
		transit<Parking>();
		return;
	}

	getMap(e.lidar_map, e.lanes);
	target_points = getTargetsByMap();

	plan(e.dynamic_objs, getSpeedByMode(2), false);

	visualization(e);
}

}//end TiEV
