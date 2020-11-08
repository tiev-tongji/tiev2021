#include "decision_state.h"

namespace TiEV{

/*Global Planning state*/
void GlobalPlanning::entry() {
	cout << "Entry GlobalPlanning State..." << endl;
	vs->print_text("State", "GlobalPlanning", 1);
}

void GlobalPlanning::exit() {
	cout << "Exit GlobalPlanning State..." << endl;
}

void GlobalPlanning::react(PlanningEvent const & e)  {
	if(e.nav_info.reliable && !task_points.empty()){
		road_manager->findReferenceRoad(e.nav_info.current_position.lon, e.nav_info.current_position.lat, task_points.front().lon, task_points.front().lat);
		cout << "Global planning successfully!" << endl;
		transit<FreeDriving>();
	}else{
		road_manager->readRoadMapFile(Config::getInstance()->roadmap_file);
		transit<NormalDriving>();
		cout << "no task!" << endl;
		return;
		//cout << "No Localization information, can not do global planning!" << endl;
	}
}

}//end TiEV
