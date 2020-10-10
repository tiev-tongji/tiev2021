#include "decision_state.h"

namespace TiEV{

/*Global Planning state*/
void GlobalRePlanning::entry() {
	cout << "Entry GlobalRePlanning State..." << endl;
	vs->print_text("State", "GlobalRePlanning", 1);
}

void GlobalRePlanning::exit() {
	cout << "Exit GlobalRePlanning State..." << endl;
}

void GlobalRePlanning::react(PlanningEvent const & e)  {
	if(e.nav_info.reliable && !task_points.empty()){
		road_manager->findReferenceRoad(e.nav_info.current_position.lon, e.nav_info.current_position.lat, task_points.front().lon, task_points.front().lat, false);
		cout << "Global rePlanning successfully!" << endl;
		transit<UturnState>();
	}else{
		road_manager->readRoadMapFile(Config::getInstance()->roadmap_file);
		transit<NormalDriving>();
		cout << "no task!" << endl;
		return;
		//cout << "No Localization information or task point, can not do global rePlanning!" << endl;
	}
}

}//end TiEV
